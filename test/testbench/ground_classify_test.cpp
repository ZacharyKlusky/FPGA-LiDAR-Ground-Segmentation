#include <iostream>
#include <vector>

extern "C" void ground_classify(
    const float *x,
    const float *y,
    const float *z,
    int num_points,
    float cell_size,
    float height_threshold,
    unsigned char *labels
);

int main() {
    // Four XY cells, including negative coordinates.
    const std::vector<float> x = {
        0.1f, 0.2f, 0.3f,
        1.1f, 1.2f,
        -0.1f, -0.2f,
        0.1f, 0.2f
    };

    const std::vector<float> y = {
        0.1f, 0.2f, 0.3f,
        0.1f, 0.2f,
        0.1f, 0.2f,
        1.1f, 1.2f
    };

    const std::vector<float> z = {
        0.0f, 0.1f, 1.5f,
        0.2f, 1.2f,
        -0.1f, 0.8f,
        0.0f, 0.2f
    };

    // 1 = ground, 0 = non-ground.
    const std::vector<unsigned char> expected = {
        1, 1, 0,
        1, 0,
        1, 0,
        1, 1
    };

    std::vector<unsigned char> actual(x.size(), 255);

    ground_classify(
        x.data(), y.data(), z.data(),
        static_cast<int>(x.size()),
        1.0f, 0.3f,
        actual.data()
    );

    int failures = 0;

    for (size_t i = 0; i < x.size(); ++i) {
        std::cout << "Point " << i
                  << " z=" << z[i]
                  << " expected=" << static_cast<int>(expected[i])
                  << " actual=" << static_cast<int>(actual[i])
                  << '\n';

        if (actual[i] != expected[i])
            ++failures;
    }

    if (failures == 0) {
        std::cout << "PASS: All ground classification tests passed\n";
        return 0;
    }

    std::cerr << "FAIL: " << failures
              << " incorrect classifications\n";
    return 1;
}
