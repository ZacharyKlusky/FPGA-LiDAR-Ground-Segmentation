# FPGA-Based LiDAR Ground Segmentation

**EECE 5698 — FPGA in the Cloud**  
**Team:** Zachary Klusky and Kevin Kuzmeski  
**Target:** AMD Alveo U280, Vitis HLS  
**Project Update 2:** November 24, 2026

## Project Overview

This project investigates the acceleration of LiDAR ground segmentation using FPGA hardware and Vitis HLS. LiDAR point clouds contain three-dimensional coordinates representing the environment. Ground segmentation separates ground returns from vegetation, buildings, and other obstacles. Our goal is to establish a correct C++ baseline, adapt suitable operations for FPGA implementation, and compare correctness, execution time, and hardware resource utilization.

We initially explored the open-source [DFKI-RIC ground segmentation](https://github.com/dfki-ric/ground_segmentation) software as a reference. Our custom implementations are separate from that repository. The current minimum-height classifier is a simplified baseline, **not** a complete implementation of GSeg3D, LineFit, or RANSAC.

## Project Update 1 — Initial Grid Processing (October 29, 2026)

The first implementation, `ground_accel`, performs **3D voxel grouping**, not ground/non-ground classification:

1. Accept arrays of 3D LiDAR coordinates.
2. Assign points to spatial grid cells using configurable dimensions.
3. Count points in occupied cells.
4. Calculate offsets into grouped point indices.
5. Organize point indices by their assigned cells.

The kernel supports up to **4,096 points per invocation**. Its standalone five-point C++ testbench passed with three occupied cells and the following expected outputs:

| Output | Expected value |
|---|---|
| Occupied cells | `3` |
| Point counts | `[3, 1, 1]` |
| Offsets | `[0, 3, 4, 5]` |
| Grouped indices | `[0, 1, 4, 2, 3]` |

Run from the repository root:

```bash
g++ -std=c++17 -O2 -Wno-unknown-pragmas \
  test/src/ground_accel.cpp test/testbench/ground_accel_test.cpp \
  -o ground_accel_test -lm
./ground_accel_test
```

Expected message: `PASS: grouping, offsets, and point indices verified`.

**Historical status clarification:** At the time of the first README, HLS synthesis had not been verified. Subsequently, HLS synthesis artifacts were produced for the **original voxel-grouping kernel**. This is not evidence that the new classification kernel was synthesized or executed on hardware.

## Project Update 2 — Ground Classification Baseline (November 24, 2026)

### Progress Since Project Update 1

We implemented a new XY-grid ground-classification kernel (`ground_classify`), added deterministic correctness tests, measured preliminary CPU execution times, and documented a failure case. The original 3D grouping kernel and the new classifier are **separate implementations**, not yet an integrated FPGA-executed pipeline.

### Input Data and Output Format

The classifier takes arrays of `float` coordinates (`x`, `y`, `z`), `num_points`, an XY `cell_size`, and a `height_threshold`. It returns one `unsigned char` label per point: **`1` = ground; `0` = obstacle/non-ground**. Maximum input size is 4,096 points. Current classification experiments use **deterministic synthetic data**, not a public LiDAR dataset. Real-data evaluation remains outstanding.

### Ground-Classification Algorithm

Source: `test/src/ground_classify.cpp`.

1. Divide the XY plane into square cells of width `cell_size` (including negative coordinates using `floor`).
2. Assign each point to a cell, maintaining the minimum Z coordinate in that cell.
3. Label a point as ground when `z_point - min_z_in_cell <= height_threshold`; otherwise label it as an obstacle.

The kernel uses fixed-size arrays and a sequential search of occupied cells. This can require **O(n²)** work in the worst case. It assumes the minimum-height return in each cell is representative of ground, which is not always true.

### HLS Architecture

The new kernel declares separate `m_axi` memory interfaces for X, Y, Z, and output labels, and `s_axilite` control interfaces for the point count, cell size, height threshold, and return/control. These pragmas describe the intended hardware interface; they **do not** demonstrate successful synthesis, timing closure, or hardware execution. No measured FPGA acceleration is claimed.

### Functional Correctness and Golden Outputs

- `test/testbench/ground_classify_test.cpp`: **9/9** predefined labels matched expected output on a small synthetic point cloud.
- `test/testbench/ground_accel_test.cpp`: original five-point grouping test passed.
- These are **CPU C++ tests**. FPGA C simulation, co-simulation, and physical execution are distinct validation milestones.

### Synthetic Benchmark Results

Benchmark source: `test/testbench/ground_classify_benchmark.cpp`. Executed on CloudLab `pc155`, compiled with `g++ -std=c++17 -O2`.

| Input points | Accuracy on synthetic data | Predicted ground | Predicted obstacles | Single-run CPU time |
|---:|---:|---:|---:|---:|
| 100 | 100.000% | 75 | 25 | 0.022 ms |
| 1,000 | 100.000% | 750 | 250 | 0.107 ms |
| 4,096 | 100.000% | 3,072 | 1,024 | 1.356 ms |

The synthetic benchmark constructs four points per XY cell: three low points designated ground and one elevated point designated obstacle. **These results do not establish accuracy on real LiDAR data.** Timings are one-shot CPU measurements, not median timings or FPGA latency; additional repetitions are needed for reliable performance comparisons.

### Known Limitation — Obstacle-Only Cell

Source: `test/testbench/ground_classify_edge_test.cpp`.

Three elevated points (Z = 1.5, 1.6, 1.7) were placed in one cell with **no ground returns**. All three should be obstacles, but the classifier labeled **3/3 as ground**. The test printed `PASS: Known limitation reproduced` because it deliberately verified this failure behavior—not because the predictions were correct. This motivates using neighboring cells, slope estimates, or more robust ground models.

### Reproducing the CPU Tests

From the repository root:

```bash
mkdir -p build_u280

g++ -std=c++17 -O2 -Wno-unknown-pragmas \
  test/src/ground_classify.cpp test/testbench/ground_classify_test.cpp \
  -o build_u280/ground_classify_test
./build_u280/ground_classify_test

g++ -std=c++17 -O2 -Wno-unknown-pragmas \
  test/src/ground_classify.cpp test/testbench/ground_classify_benchmark.cpp \
  -o build_u280/ground_classify_benchmark
./build_u280/ground_classify_benchmark

g++ -std=c++17 -O2 -Wno-unknown-pragmas \
  test/src/ground_classify.cpp test/testbench/ground_classify_edge_test.cpp \
  -o build_u280/ground_classify_edge_test
./build_u280/ground_classify_edge_test
```

### FPGA Development Status

The target is the AMD Alveo U280, part `xcu280-fsvh2892-2L-e`.
The earlier grouping-kernel HLS configuration specifies
200 MHz (5 ns); this is a requested clock, not achieved
timing for the classifier.

The classifier has HLS interface pragmas but has not
completed HLS synthesis, co-simulation, linking or
physical execution.

The following environment was verified on November 24, 2026:

| Environment item | Observed status |
|---|---|
| OCT node | pc155; Ubuntu 22.04.2 LTS |
| XRT | 2023.2 / 2.16.204 |
| U280 shell | xilinx_u280_gen3x16_xdma_base_1; ready |
| Development platform | xilinx_u280_gen3x16_xdma_1_202211_1 |
| Vitis / HLS | 2024.2 available; 2023.1/2023.2 not found in searched paths |
| Vivado 2024.2 | Starts after installing libtinfo5; U280 part queries empty |

The initial Vivado startup failed because `libtinfo.so.5`
was missing. Installing `libtinfo5` resolved startup.
A subsequent batch Tcl script executed
`get_parts -quiet xcu280*` and
`get_parts -quiet xcu280-fsvh2892-2L-e`;
both returned empty results.

The available 2024.2 installation therefore does not
expose the required U280 part. Searches of the shared tool
directories found only 2024.2 Vitis/HLS executables.
Earlier Vivado 2023.1 device support did not provide a
complete matching Vitis toolchain.

The physical card is detected and ready, but the
compiler/device-support issue prevents a verified
classifier build in the current environment.

No classifier FPGA resource utilization, achieved clock,
hardware latency, throughput or speedup is claimed.
Prior synthesis artifacts for `ground_accel` do not
establish synthesis or execution of `ground_classify`.

### Algorithm-Level Design-Space Exploration

A parameter sweep was conducted to evaluate the effect of XY cell size and height threshold on classification accuracy and CPU execution time.

The experiment uses a fixed synthetic dataset containing 4,096 points, including 3,072 ground points and 1,024 obstacle points. The scene includes sloped terrain and obstacles at varying heights.

Four cell sizes (0.25, 0.5, 1.0, and 2.0 m) and four height thresholds (0.1, 0.2, 0.3, and 0.5 m) were evaluated, producing 16 configurations. Each configuration was executed 20 times, and the median CPU execution time was recorded.

#### Representative Results

| Cell Size (m) | Threshold (m) | Accuracy (%) | False Ground | False Obstacle | Median CPU (ms) |
|---:|---:|---:|---:|---:|---:|
| 0.25 | 0.10 | 100.000 | 0 | 0 | 2.984 |
| 0.50 | 0.10 | 100.000 | 0 | 0 | 1.696 |
| 1.00 | 0.10 | 100.000 | 0 | 0 | 0.957 |
| 1.00 | 0.20 | 100.000 | 0 | 0 | 0.956 |
| 1.00 | 0.30 | 96.875 | 128 | 0 | 0.956 |
| 2.00 | 0.10 | 62.500 | 0 | 1536 | 0.287 |
| 2.00 | 0.30 | 98.438 | 64 | 0 | 0.287 |

The complete results for all 16 configurations are available in [results/ground_classify_sweep.txt](results/ground_classify_sweep.txt).

#### Analysis

- Increasing cell size reduces the number of occupied cells and therefore reduces the computational cost of the sequential cell-search algorithm.
- At a fixed height threshold of 0.3 m, increasing cell size from 0.25 m to 2.0 m reduced median CPU execution time from 2.856 ms to 0.287 ms, approximately a 9.95x improvement.
- The fastest configuration achieving 100% classification accuracy used a 1.0 m cell size and a 0.2 m height threshold.
- A 2.0 m cell size and 0.3 m threshold achieved 98.438% accuracy with a median execution time of 0.287 ms, approximately 3.33x faster than the fastest perfect-accuracy configuration.
- Excessively large height thresholds can cause obstacles to be incorrectly classified as ground.
- Large cells combined with small height thresholds can cause legitimate ground points on sloped terrain to be incorrectly classified as obstacles.

These results demonstrate an accuracy-performance trade-off for the current minimum-height classification algorithm. The optimal parameters are specific to this synthetic dataset and require further validation using real-world LiDAR data.

These measurements represent CPU execution only and do not establish FPGA latency, throughput, or acceleration.

#### Reproducing the Parameter Sweep

```bash
mkdir -p build_u280 results

g++ -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas \
    test/src/ground_classify.cpp \
    test/testbench/ground_classify_sweep.cpp \
    -o build_u280/ground_classify_sweep

./build_u280/ground_classify_sweep \
    | tee results/ground_classify_sweep.txt
```

### Real-World Validation — SemanticKITTI

#### Dataset and Evaluation Methodology

The ground-classification kernel was evaluated using an annotated
SemanticKITTI sample distributed by Supervisely. The sample contains
ten consecutive LiDAR scans (frames 000000–000009) from sequence 00.

All ten scans passed dataset integrity checks, with matching point
and semantic-label counts.

| Dataset characteristic | Count |
|---|---:|
| LiDAR scans | 10 |
| Total points | 1,235,429 |
| Ground points | 625,803 |
| Non-ground points | 589,636 |
| Excluded points | 19,990 |
| Evaluated points | 1,215,439 |

SemanticKITTI point clouds contain four 32-bit floating-point values
per point: X, Y, Z, and intensity. Semantic class IDs are extracted
from the lower 16 bits of each 32-bit annotation.

For binary ground classification, the following semantic classes
are considered ground:

- 40: Road
- 44: Parking
- 48: Sidewalk
- 49: Other-ground
- 60: Lane marking
- 72: Terrain

Classes 0 (unlabeled) and 1 (outlier) are excluded from scoring.
All other observed semantic classes are considered non-ground.
Excluded points remain available to the classifier during processing,
preventing ground-truth information from influencing input selection.

#### Spatial Batching

The existing kernel supports a maximum of 4,096 points per invocation.
To process complete LiDAR scans without modifying the kernel, the CPU
evaluation program groups points by their XY grid-cell coordinates
and constructs batches containing complete cells.

This preserves each cell's minimum-height calculation across batch
boundaries. Configurations containing an individual cell larger than
the kernel capacity are rejected explicitly.

The evaluation program is implemented in
`test/testbench/ground_classify_real.cpp`.

#### Real-World Baseline and Parameter Optimization

The original baseline uses a 1.0 m cell size and a 0.3 m height
threshold. A parameter sweep evaluates four cell sizes
(0.25, 0.5, 1.0, and 2.0 m) and four height thresholds
(0.1, 0.2, 0.3, and 0.5 m).

All 16 configurations successfully processed all ten scans,
representing 19,766,864 point classifications, including ignored
points.

The best tested configuration by accuracy, F1 score, and ground IoU
uses a 2.0 m cell size and a 0.2 m height threshold.

| Metric | Original baseline | Best tested configuration |
|---|---:|---:|
| Cell size | 1.0 m | 2.0 m |
| Height threshold | 0.3 m | 0.2 m |
| Accuracy | 89.608% | 93.677% |
| Ground precision | 83.279% | 90.797% |
| Ground recall | 99.869% | 97.615% |
| Ground F1 score | 90.823% | 94.082% |
| Ground IoU | 83.188% | 88.826% |
| True positives | 624,986 | 610,875 |
| True negatives | 464,145 | 527,716 |
| False positives | 125,491 | 61,920 |
| False negatives | 817 | 14,928 |
| Kernel invocations | 320 | 345 |
| Mean kernel CPU time per scan | 3.506 ms | 1.854 ms |
| Mean total processing time per scan | 9.149 ms | 6.162 ms |

Timing values in this table are taken from the same parameter sweep.
Classification metrics are aggregated across all evaluated points.
Total processing time includes spatial partitioning, kernel execution,
and metric calculation, but excludes dataset file loading.

The optimized parameter configuration reduces false positives by
50.66% and total classification errors by 39.16%. Accuracy improves
by 4.069 percentage points, while mean total processing time decreases
by approximately 32.6%.

These improvements were achieved without modifying the classification
kernel.

#### Representative Design-Space Results

| Cell (m) | Threshold (m) | Accuracy | F1 score | CPU ms/scan |
|---:|---:|---:|---:|---:|
| 0.25 | 0.1 | 89.835% | 90.963% | 25.623 |
| 0.5 | 0.1 | 92.682% | 93.244% | 15.656 |
| 1.0 | 0.1 | 93.112% | 93.387% | 9.116 |
| 1.0 | 0.2 | 92.530% | 93.210% | 10.209 |
| 1.0 | 0.3 | 89.608% | 90.823% | 9.149 |
| 2.0 | 0.1 | 86.488% | 85.684% | 6.203 |
| 2.0 | 0.2 | 93.677% | 94.082% | 6.162 |
| 2.0 | 0.3 | 91.963% | 92.723% | 6.159 |

Complete experimental results are available in:

- `results/ground_classify_real_baseline.txt`
- `results/ground_classify_real_optimized.txt`
- `results/ground_classify_real_sweep.txt`

#### Engineering Analysis

Increasing the cell size generally reduces the number of unique
cells searched by the current sequential lookup implementation,
reducing CPU execution time. Larger cells may also reduce the
occurrence of obstacle-only cells, although this mechanism has
not yet been independently quantified.

Increasing the height threshold improves ground recall at the
expense of precision. Conversely, an excessively restrictive
threshold can misclassify uneven or sloped ground as non-ground.

The real-world results demonstrate that parameter selection based
solely on synthetic data is insufficient. The best tested real-world
configuration differs from the highest-accuracy synthetic
configurations.

The standalone baseline and optimized evaluations reproduced the
same classification metrics as the parameter sweep. CPU timings
varied between executions, so repeated timing trials will be needed
for a more rigorous performance characterization.

#### Implications for FPGA Acceleration

For the original baseline in the real-world parameter sweep,
approximately 38.3% of total processing time is spent executing
the classification kernel. The remaining time is attributable to
spatial partitioning, data preparation, metric calculation, and
other evaluation overhead.

Under Amdahl's law, infinitely accelerating only the classifier
would provide a theoretical overall speedup of approximately
1.62x for this measured workload, assuming all other overhead
remains unchanged.

This motivates optimization of both the FPGA kernel and the
host-side processing pipeline. Future hardware measurements
must also account for host-to-device and device-to-host transfers.

These results are CPU measurements, not measured FPGA performance.

#### Published Reference and Limitations

The original GSeg3D repository reports precision of approximately
96–99%, recall of 85–91%, F1 scores of 92–94%, and an average CPU
runtime of approximately 48 ms per scan on SemanticKITTI.

Our best tested configuration achieves an F1 score of 94.082%
on the downloaded sample. However, differences in dataset coverage,
hardware, ground-truth mapping, algorithm complexity, and timing
methodology prevent a direct performance comparison.

The ten evaluated scans are consecutive frames from a single
sequence. The selected parameters have not yet been validated on
an independent dataset. The obstacle-only cell limitation also
remains unresolved.

#### Reproducing the Real-World Evaluation

Download the annotated sample:

```bash
mkdir -p datasets/semantickitti

curl -fL \\
    https://github.com/supervisely-ecosystem/demo-semantic-kitti-pointcloud-episodes-annotated/releases/download/v1.0.0/project-example.zip \\
    -o datasets/semantickitti/project-example.zip

unzip -q datasets/semantickitti/project-example.zip \\
    -d datasets/semantickitti/
```

Compile the CPU evaluation program:

```bash
mkdir -p build_u280

g++ -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas \\
    test/src/ground_classify.cpp \\
    test/testbench/ground_classify_real.cpp \\
    -o build_u280/ground_classify_real
```

Run the original baseline:

```bash
./build_u280/ground_classify_real \\
    datasets/semantickitti/dataset/sequences/00 \\
    1.0 0.3
```

Run the best tested configuration:

```bash
./build_u280/ground_classify_real \\
    datasets/semantickitti/dataset/sequences/00 \\
    2.0 0.2
```

Reproduce the parameter sweep:

```bash
for cell in 0.25 0.5 1.0 2.0; do
    for threshold in 0.1 0.2 0.3 0.5; do
        echo "Cell: $cell | Threshold: $threshold"
        ./build_u280/ground_classify_real \\
            datasets/semantickitti/dataset/sequences/00 \\
            "$cell" "$threshold" | grep '^TOTAL,'
    done
done
```

#### Dataset Attribution

- [SemanticKITTI Dataset](https://semantic-kitti.org/)
- [Supervisely Annotated SemanticKITTI Sample](https://github.com/supervisely-ecosystem/demo-semantic-kitti-pointcloud-episodes-annotated)
- Behley et al., *SemanticKITTI: A Dataset for Semantic Scene
  Understanding of LiDAR Sequences*, ICCV, 2019.
- Geiger, Lenz, and Urtasun, *Are We Ready for Autonomous Driving?
  The KITTI Vision Benchmark Suite*, CVPR, 2012.

The sample is distributed under the Creative Commons
Attribution-NonCommercial-ShareAlike 4.0 license, as stated by
the sample distributor. Raw dataset files are excluded from this
repository.

### Design-Space Exploration and Next Steps

Algorithm-level design-space exploration has been completed for
both synthetic and real-world LiDAR data. The experiments quantify
the effects of grid-cell size and height threshold on classification
accuracy, precision, recall, F1 score, and CPU execution time.

The best tested real-world configuration uses a 2.0 m cell size
and a 0.2 m height threshold, achieving 93.677% accuracy and
a 94.082% ground F1 score on the evaluated SemanticKITTI sample.

Remaining development priorities include:

- Replacing sequential cell lookup with hardware-friendly indexing.
- Exploring loop pipelining and on-chip memory optimization.
- Improving host-side spatial partitioning and data preparation.
- Addressing the obstacle-only cell classification limitation.
- Validating parameter selection on independent LiDAR sequences.
- Conducting repeated CPU timing trials.
- Completing HLS synthesis and analyzing resource utilization,
  latency, and initiation intervals.
- Implementing and validating FPGA host-to-device communication.
- Measuring FPGA kernel and end-to-end execution performance.
- Comparing CPU and FPGA implementations using identical workloads.

FPGA synthesis and hardware validation remain outstanding due to
the documented toolchain and platform compatibility issues.
No FPGA speedup or resource utilization claims are made at this stage.

### Individual Contributions

This project is a collaborative effort between Zachary Klusky and Kevin Kuzmeski. For Project Update 2, Zachary primarily focused on the development and validation of the ground-classification algorithm, synthetic and real-world evaluation, parameter optimization, and analysis of classification accuracy and computational performance. Additional efforts included investigating the OCT development environment and preparing the project documentation. Moving forward, the primary focus of the project will be FPGA implementation, hardware validation, and architectural optimization.

### References and Software Attribution

- [DFKI-RIC ground_segmentation](https://github.com/dfki-ric/ground_segmentation) — open-source reference implementation, included in the `ground_segmentation/` directory.
- [AMD Vitis HLS documentation](https://docs.amd.com/r/en-US/ug1399-vitis-hls) — HLS interface and optimization concepts.

The project's `test/src/ground_accel.cpp` and `test/src/ground_classify.cpp` are distinct custom kernels; the current classifier does not reproduce the complete DFKI-RIC algorithm. Other reused code or dependencies should be credited explicitly as identified.

### AI-Assisted Development Disclosure

ChatGPT was used to assist with C++ kernel debugging,
evaluation code, FPGA toolchain troubleshooting, and
reviewing this report. Code and experiments were run in
the OCT environment, and reported test results and CPU
timings are based on observed terminal output.

---

## Original Software Reference — Setup and Build

The reference repository is located in `ground_segmentation/`: https://github.com/dfki-ric/ground_segmentation

On Ubuntu, install the required dependencies:

```bash
sudo apt update
sudo apt install cmake libpcl-dev libeigen3-dev libgtest-dev libnanoflann-dev
```

Build from the repository root:

```bash
cd ground_segmentation
/usr/bin/cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
/usr/bin/cmake --build build -j"$(nproc)"
```

The explicit `/usr/bin/cmake` uses the system CMake rather than a Vitis-bundled version. The original reference implementation and the custom HLS-oriented kernels are built separately. The HLS configuration in `test/ground_segmentation_hls/` may contain machine-specific absolute paths that need adjustment for another environment.

## Repository Layout

| Path | Purpose |
|---|---|
| `ground_segmentation/` | External DFKI-RIC reference code |
| `test/src/ground_accel.cpp` | Original 3D voxel grouping kernel |
| `test/testbench/ground_accel_test.cpp` | Original grouping correctness test |
| `test/src/ground_classify.cpp` | New XY-grid classification kernel |
| `test/testbench/ground_classify_test.cpp` | Classification correctness test |
| `test/testbench/ground_classify_benchmark.cpp` | Synthetic accuracy/CPU benchmark |
| `test/testbench/ground_classify_edge_test.cpp` | Known failure-case demonstration |
| `test/ground_segmentation_hls/` | Existing HLS project/configuration and artifacts |

**Status as of November 24, 2026:** CPU baseline validated on synthetic tests; classification FPGA execution remains unverified. Future results should be appended with dates and corresponding Git commits.
