#include <cmath>

static constexpr int MAX_POINTS = 4096;

extern "C" void ground_classify(
    const float *x,
    const float *y,
    const float *z,
    int num_points,
    float cell_size,
    float height_threshold,
    unsigned char *labels
) {
#pragma HLS INTERFACE m_axi port=x offset=slave bundle=gmem0
#pragma HLS INTERFACE m_axi port=y offset=slave bundle=gmem1
#pragma HLS INTERFACE m_axi port=z offset=slave bundle=gmem2
#pragma HLS INTERFACE m_axi port=labels offset=slave bundle=gmem3
#pragma HLS INTERFACE s_axilite port=num_points bundle=control
#pragma HLS INTERFACE s_axilite port=cell_size bundle=control
#pragma HLS INTERFACE s_axilite port=height_threshold bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control

    if (num_points <= 0 || num_points > MAX_POINTS ||
        cell_size <= 0.0f || height_threshold < 0.0f)
        return;

    int cell_x[MAX_POINTS];
    int cell_y[MAX_POINTS];
    float cell_min_z[MAX_POINTS];
    int point_cell[MAX_POINTS];

    int num_cells = 0;

    // Pass 1: Find minimum height in each XY cell.
    for (int i = 0; i < num_points; ++i) {
        int cx = static_cast<int>(std::floor(x[i] / cell_size));
        int cy = static_cast<int>(std::floor(y[i] / cell_size));

        int found = -1;
        for (int j = 0; j < num_cells; ++j) {
            if (cell_x[j] == cx && cell_y[j] == cy) {
                found = j;
                break;
            }
        }

        if (found == -1) {
            found = num_cells++;
            cell_x[found] = cx;
            cell_y[found] = cy;
            cell_min_z[found] = z[i];
        } else if (z[i] < cell_min_z[found]) {
            cell_min_z[found] = z[i];
        }

        point_cell[i] = found;
    }

    // Pass 2: Classify points relative to local minimum.
    // 1 = ground, 0 = non-ground.
    for (int i = 0; i < num_points; ++i) {
        float min_height = cell_min_z[point_cell[i]];
        labels[i] = (z[i] - min_height <= height_threshold) ? 1 : 0;
    }
}
