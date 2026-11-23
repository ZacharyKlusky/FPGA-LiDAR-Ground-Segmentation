#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

extern "C" void ground_classify(
    const float *x, const float *y, const float *z,
    int num_points, float cell_size, float height_threshold,
    unsigned char *labels
);

void benchmark(int n, float cell_size) {
    std::vector<float> x(n), y(n), z(n);
    std::vector<unsigned char> expected(n), actual(n, 255);

    // Four points per XY cell:
    // three ground points and one elevated obstacle.
    // All coordinates are deterministic.
    for (int i = 0; i < n; ++i) {
        int cell = i / 4;
        int position = i % 4;

        int cx = cell % 32;
        int cy = cell / 32;

        x[i] = (cx + 0.1f + 0.1f * position) * cell_size;
        y[i] = (cy + 0.1f + 0.1f * position) * cell_size;

        if (position == 3) {
            z[i] = 1.5f;
            expected[i] = 0;
        } else {
            z[i] = 0.05f * position;
            expected[i] = 1;
        }
    }

    auto start = std::chrono::steady_clock::now();

    ground_classify(
        x.data(), y.data(), z.data(),
        n, cell_size, 0.3f, actual.data()
    );

    auto end = std::chrono::steady_clock::now();

    int correct = 0;
    int ground_count = 0;
    int obstacle_count = 0;

    for (int i = 0; i < n; ++i) {
        if (actual[i] == expected[i])
            ++correct;

        if (actual[i] == 1)
            ++ground_count;
        else if (actual[i] == 0)
            ++obstacle_count;
    }

    double elapsed_ms =
        std::chrono::duration<double, std::milli>(
            end - start
        ).count();

    double accuracy = 100.0 * correct / n;

    std::cout << std::fixed << std::setprecision(3)
              << "Points: " << std::setw(4) << n
              << " | Accuracy: " << accuracy << "%"
              << " | Ground: " << ground_count
              << " | Obstacles: " << obstacle_count
              << " | CPU time: " << elapsed_ms << " ms"
              << '\n';
}

int main() {
    std::cout << "LiDAR Ground Classification Benchmark\n";
    std::cout << "-------------------------------------\n";

    benchmark(100, 1.0f);
    benchmark(1000, 1.0f);
    benchmark(4096, 1.0f);

    return 0;
}
