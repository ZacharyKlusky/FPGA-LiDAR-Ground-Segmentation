#include <cstdio>

static constexpr int MAX_POINTS = 4096;

extern "C" {
void ground_accel(
    const float point_x[MAX_POINTS],
    const float point_y[MAX_POINTS],
    const float point_z[MAX_POINTS],
    int num_points,
    float cell_size_x,
    float cell_size_y,
    float cell_size_z,
    int point_cell[MAX_POINTS],
    int cell_x[MAX_POINTS],
    int cell_y[MAX_POINTS],
    int cell_z[MAX_POINTS],
    int cell_point_count[MAX_POINTS],
    int cell_point_offsets[MAX_POINTS + 1],
    int point_indices[MAX_POINTS],
    int &num_cells
);
}

int main() {
    float x[MAX_POINTS] = {0.1f, 0.5f, 2.1f, -0.1f, 1.0f};
    float y[MAX_POINTS] = {0.1f, 0.5f, 0.1f, 0.1f, 0.5f};
    float z[MAX_POINTS] = {0.1f, 0.5f, 0.1f, 0.1f, 0.5f};

    int point_cell[MAX_POINTS] = {};
    int cell_x[MAX_POINTS] = {};
    int cell_y[MAX_POINTS] = {};
    int cell_z[MAX_POINTS] = {};
    int cell_point_count[MAX_POINTS] = {};
    int cell_point_offsets[MAX_POINTS + 1] = {};
    int point_indices[MAX_POINTS] = {};
    int num_cells = 0;

    ground_accel(
        x, y, z, 5, 2.0f, 2.0f, 10.0f,
        point_cell, cell_x, cell_y, cell_z,
        cell_point_count, cell_point_offsets,
        point_indices, num_cells
    );

    if (num_cells != 3 ||
        cell_point_count[0] != 3 ||
        cell_point_count[1] != 1 ||
        cell_point_count[2] != 1) {
        std::printf("FAIL: incorrect cell grouping\n");
        return 1;
    }

    if (cell_point_offsets[0] != 0 ||
        cell_point_offsets[1] != 3 ||
        cell_point_offsets[2] != 4 ||
        cell_point_offsets[3] != 5) {
        std::printf("FAIL: incorrect cell offsets\n");
        return 1;
    }

    // Cells are created in first-seen order.
    const int expected_indices[5] = {0, 1, 4, 2, 3};

    for (int i = 0; i < 5; ++i) {
        if (point_indices[i] != expected_indices[i]) {
            std::printf("FAIL: point_indices[%d] = %d\n",
                        i, point_indices[i]);
            return 1;
        }
    }

    std::printf("PASS: grouping, offsets, and point indices verified\n");
    return 0;
}