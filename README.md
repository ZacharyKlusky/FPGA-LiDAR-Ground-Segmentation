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

**Target hardware:** AMD Alveo U280. The CloudLab experiment detected a ready U280. The original voxel-grouping kernel has prior Vitis HLS synthesis artifacts, but the new `ground_classify` kernel has **not** been verified through HLS synthesis, emulation, or hardware execution. The expected Vitis 2023.1/2023.2 installations were unavailable at the shared configured path; attempting the available 2024.2 toolchain encountered target-device support problems. The team has requested guidance about the supported build environment.

**Outstanding:** classification-kernel HLS synthesis, hardware resource and timing reports, software/hardware emulation, U280 execution, and matched CPU-versus-FPGA comparisons. No FPGA speedup or hardware accuracy is reported.

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

### Design-Space Exploration and Next Steps

The current measured baseline uses `cell_size = 1.0` and `height_threshold = 0.3` in the synthetic benchmark. A controlled parameter sweep has **not** yet been performed. Planned exploration includes different cell sizes and thresholds; testing sparse and sloped terrain; comparing linear cell search with more hardware-friendly indexing; loop pipelining and memory optimization; real-world LiDAR datasets; HLS resource/latency reports; and hardware acceleration once the toolchain is available. This list is a plan, **not completed experimental evidence**.

### Team Contributions

This is a two-person project. Zachary Klusky worked on the new XY-grid ground-classification baseline, deterministic CPU tests, synthetic benchmarks, obstacle-only failure analysis, and Project Update 2 documentation/toolchain investigation. **Kevin Kuzmeski's exact contributions and the ownership of the original grouping/HLS work must be confirmed with Kevin before final submission.** This placeholder avoids attributing unverified work.

### References and Software Attribution

- [DFKI-RIC ground_segmentation](https://github.com/dfki-ric/ground_segmentation) — open-source reference implementation, included in the `ground_segmentation/` directory.
- [AMD Vitis HLS documentation](https://docs.amd.com/r/en-US/ug1399-vitis-hls) — HLS interface and optimization concepts.

The project's `test/src/ground_accel.cpp` and `test/src/ground_classify.cpp` are distinct custom kernels; the current classifier does not reproduce the complete DFKI-RIC algorithm. Other reused code or dependencies should be credited explicitly as identified.

### AI-Assisted Development Disclosure

AI assisted with C++ kernel development, debugging, FPGA toolchain troubleshooting, and documentation. All stated test results and CPU timings are taken from observed terminal output, not AI-estimated performance. Independent FPGA synthesis and hardware validation remain outstanding.

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
