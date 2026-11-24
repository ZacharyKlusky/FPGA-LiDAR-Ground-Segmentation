#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

extern "C" void ground_classify(
    const float *x, const float *y, const float *z,
    int num_points, float cell_size,
    float height_threshold, unsigned char *labels
);

int main() {
    const int N = 4096;
    const int RUNS = 20;

    std::vector<float> x(N), y(N), z(N);
    std::vector<unsigned char> expected(N);
    std::vector<unsigned char> actual(N);

    // Fixed synthetic scene with sloped ground
    // and obstacles at different heights.
    for (int i = 0; i < N; ++i) {
        int cell = i / 4;
        int pos = i % 4;

        x[i] = (cell % 32) + 0.1f + 0.2f * pos;
        y[i] = (cell / 32) + 0.1f + 0.2f * pos;

        float ground_height =
            0.08f * x[i] + 0.04f * y[i];

        if (pos == 3) {
            z[i] = ground_height
                 + 0.2f + 0.15f * (cell % 8);
            expected[i] = 0;
        } else {
            z[i] = ground_height + 0.02f * pos;
            expected[i] = 1;
        }
    }

    float cell_sizes[] = {0.25f, 0.5f, 1.0f, 2.0f};
    float thresholds[] = {0.1f, 0.2f, 0.3f, 0.5f};

    std::cout << "Ground Classification Parameter Sweep\n";
    std::cout << "Points: " << N
              << " | Runs per configuration: "
              << RUNS << "\n\n";

    std::cout << "Cell(m),Threshold(m),Accuracy(%),"
              << "FalseGround,FalseObstacle,MedianCPU(ms)\n";

    for (float cell : cell_sizes) {
        for (float threshold : thresholds) {
            std::vector<double> times;

            for (int r = 0; r < RUNS; ++r) {
                auto start =
                    std::chrono::steady_clock::now();

                ground_classify(
                    x.data(), y.data(), z.data(),
                    N, cell, threshold, actual.data()
                );

                auto end =
                    std::chrono::steady_clock::now();

                times.push_back(
                    std::chrono::duration<double,
                    std::milli>(end - start).count()
                );
            }

            int correct = 0;
            int false_ground = 0;
            int false_obstacle = 0;

            for (int i = 0; i < N; ++i) {
                if (actual[i] == expected[i])
                    ++correct;
                else if (actual[i] == 1)
                    ++false_ground;
                else
                    ++false_obstacle;
            }

            std::sort(times.begin(), times.end());

            double median =
                (times[RUNS / 2 - 1]
                + times[RUNS / 2]) / 2.0;

            double accuracy = 100.0 * correct / N;

            std::cout << std::fixed
                      << std::setprecision(3)
                      << cell << ","
                      << threshold << ","
                      << accuracy << ","
                      << false_ground << ","
                      << false_obstacle << ","
                      << median << "\n";
        }
    }

    return 0;
}
