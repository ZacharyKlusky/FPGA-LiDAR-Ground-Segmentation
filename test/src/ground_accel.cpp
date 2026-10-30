#include <cmath>

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
) {
#pragma HLS INTERFACE m_axi port=point_x offset=slave bundle=gmem0
#pragma HLS INTERFACE m_axi port=point_y offset=slave bundle=gmem1
#pragma HLS INTERFACE m_axi port=point_z offset=slave bundle=gmem2
#pragma HLS INTERFACE m_axi port=point_cell offset=slave bundle=gmem3
#pragma HLS INTERFACE m_axi port=cell_x offset=slave bundle=gmem4
#pragma HLS INTERFACE m_axi port=cell_y offset=slave bundle=gmem5
#pragma HLS INTERFACE m_axi port=cell_z offset=slave bundle=gmem6
#pragma HLS INTERFACE m_axi port=cell_point_count offset=slave bundle=gmem7
#pragma HLS INTERFACE m_axi port=cell_point_offsets offset=slave bundle=gmem8
#pragma HLS INTERFACE m_axi port=point_indices offset=slave bundle=gmem9

#pragma HLS INTERFACE s_axilite port=num_points 
#pragma HLS INTERFACE s_axilite port=cell_size_x
#pragma HLS INTERFACE s_axilite port=cell_size_y
#pragma HLS INTERFACE s_axilite port=cell_size_z
#pragma HLS INTERFACE s_axilite port=num_cells 
#pragma HLS INTERFACE s_axilite port=return 

    num_cells = 0;

    if (num_points < 0 || num_points > MAX_POINTS ||
        cell_size_x <= 0.0f ||
        cell_size_y <= 0.0f ||
        cell_size_z <= 0.0f) {
        return;
    }

    for (int c = 0; c < MAX_POINTS; ++c) {
#pragma HLS PIPELINE II=1
        cell_point_count[c] = 0;
    }

    // Pass 1: assign points to unique occupied cells.
    for (int i = 0; i < num_points; ++i) {
        const int x = static_cast<int>(
            std::floor(point_x[i] / cell_size_x));
        const int y = static_cast<int>(
            std::floor(point_y[i] / cell_size_y));
        const int z = static_cast<int>(
            std::floor(point_z[i] / cell_size_z));

        int found = -1;

        for (int c = 0; c < num_cells; ++c) {
            if (cell_x[c] == x &&
                cell_y[c] == y &&
                cell_z[c] == z) {
                found = c;
                break;
            }
        }

        if (found < 0) {
            found = num_cells;
            cell_x[found] = x;
            cell_y[found] = y;
            cell_z[found] = z;
            ++num_cells;
        }

        point_cell[i] = found;
        ++cell_point_count[found];
    }

    // Pass 2: calculate the starting offset for each cell.
    int offset = 0;
    int write_pos[MAX_POINTS];

    for (int c = 0; c < num_cells; ++c) {
        cell_point_offsets[c] = offset;
        write_pos[c] = offset;
        offset += cell_point_count[c];
    }

    // Sentinel offset: the end of the final cell's range.
    cell_point_offsets[num_cells] = offset;

    // Pass 3: place each original point index in its cell's range.
    for (int i = 0; i < num_points; ++i) {
        const int c = point_cell[i];
        point_indices[write_pos[c]] = i;
        ++write_pos[c];
    }
}
}