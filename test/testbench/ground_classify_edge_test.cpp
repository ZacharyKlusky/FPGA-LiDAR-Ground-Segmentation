#include <iostream>

extern "C" void ground_classify(
    const float *x, const float *y, const float *z,
    int num_points, float cell_size, float height_threshold,
    unsigned char *labels
);

int main() {
    // One cell containing only elevated obstacle points.
    float x[] = {0.1f, 0.2f, 0.3f};
    float y[] = {0.1f, 0.2f, 0.3f};
    float z[] = {1.5f, 1.6f, 1.7f};

    unsigned char labels[3] = {255, 255, 255};

    ground_classify(x, y, z, 3, 1.0f, 0.3f, labels);

    std::cout << "Obstacle-only cell test\n";

    int false_ground = 0;

    for (int i = 0; i < 3; ++i) {
        std::cout << "Point " << i
                  << " | height=" << z[i]
                  << " | predicted="
                  << (labels[i] == 1 ? "ground" : "obstacle")
                  << '\n';

        if (labels[i] == 1)
            ++false_ground;
    }

    std::cout << "False ground classifications: "
              << false_ground << "/3\n";

    // Expected demonstration of the baseline limitation.
    if (false_ground == 3) {
        std::cout << "PASS: Known limitation reproduced\n";
        return 0;
    }

    std::cout << "FAIL: Unexpected classifier behavior\n";
    return 1;
}
