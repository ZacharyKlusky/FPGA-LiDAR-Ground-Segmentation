# FPGA-Based LiDAR Ground Segmentation

## Project Overview

This project investigates the acceleration of LiDAR ground segmentation using FPGA hardware and Vitis HLS. LiDAR point clouds consist of three-dimensional coordinates representing the surrounding environment. Ground segmentation separates points belonging to the ground from those representing objects such as vegetation, buildings, and other obstacles.

Our goal is to implement a grid-based ground-segmentation pipeline and evaluate the potential performance improvements achieved through FPGA acceleration compared with a software implementation.

For Project Update 1, we focused on establishing the software reference implementation and developing the initial grid-processing stage for FPGA acceleration.

## Project Update 1: Completed Work

We cloned and built the open-source ground-segmentation repository from DFKI-RIC, which provides the software reference for our project.

We also developed an initial C++ accelerator function, `ground_accel`, intended for Vitis HLS. This implementation currently performs the following operations:

1. Accepts arrays of 3D LiDAR point coordinates.
2. Assigns points to spatial grid cells using configurable cell dimensions.
3. Identifies occupied cells and counts the points in each cell.
4. Calculates offsets identifying each cell's range of points.
5. Organizes point indices by their assigned cells.

The current implementation supports up to 4,096 points per invocation.

We created a standalone C++ testbench containing five predefined points and expected results. The testbench successfully compiled and passed on October 29, 2026.

This verifies the grid-processing functionality for the provided test case. Full ground classification and FPGA synthesis remain future work.

## Submitted Files

- `ground_segmentation/` — Original DFKI-RIC ground-segmentation repository.
- `test/src/ground_accel.cpp` — HLS-oriented grid-processing implementation.
- `test/testbench/ground_accel_test.cpp` — Testbench containing sample input and expected output.
- `test/ground_segmentation_hls/hls_config.cfg` — Vitis HLS configuration.
- `README.md` — Project documentation and instructions.

## Running the Grid-Processing Implementation

**Requirements:** Linux environment with a C++17-compatible `g++` compiler.

From the `Ground_Segmentation` project directory, run:

```bash
g++ -std=c++17 -O2 \
  test/src/ground_accel.cpp \
  test/testbench/ground_accel_test.cpp \
  -o ground_accel_test -lm

./ground_accel_test
```

**Expected output:**

```text
PASS: grouping, offsets, and point indices verified
```

The testbench supplies five points and divides them into three occupied grid cells.

Expected results:

- Number of occupied cells: `3`
- Cell point counts: `[3, 1, 1]`
- Cell offsets: `[0, 3, 4, 5]`
- Grouped point indices: `[0, 1, 4, 2, 3]`

The testbench compares the computed results against these expected values and returns a nonzero exit code if a verification check fails.

## Current Status and Future Work

The grid-processing implementation passes its standalone C++ testbench. Successful Vitis HLS C simulation and synthesis have not yet been demonstrated.

The remaining stages include:

1. Per-cell statistics and principal component analysis (PCA).
2. Plane fitting and slope estimation.
3. Ground-region expansion.
4. End-to-end ground-segmentation validation.
5. HLS optimization and evaluation of latency, throughput, and FPGA resource utilization.

The objective of Project Update 1 is to establish a reproducible starting point for these subsequent development stages.

---

## Software Reference Implementation: Setup and Build Instructions

The original software reference implementation is based on the following open-source repository:

https://github.com/dfki-ric/ground_segmentation

The repository was cloned into the `ground_segmentation/` directory.

### Dependencies

On Ubuntu, install the required packages:

```bash
sudo apt update
sudo apt install cmake libpcl-dev libeigen3-dev libgtest-dev libnanoflann-dev
```

### Building the Reference Implementation

From the `Ground_Segmentation` project directory:

```bash
cd ground_segmentation
/usr/bin/cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
/usr/bin/cmake --build build -j"$(nproc)"
```

The explicit `/usr/bin/cmake` path selects the system CMake installation rather than a version bundled with Vitis.

These commands build the original software implementation. The separate grid-processing accelerator and its testbench can be compiled and executed using the instructions earlier in this README.

### HLS Development Status

The initial grid-processing accelerator has been written in C++ with Vitis HLS interface directives.

Standalone C++ testing has passed. HLS C simulation and synthesis are not yet verified.

The project is being developed using Vitis 2023.2, targeting the AMD Alveo U280 FPGA.

The HLS configuration currently contains absolute source paths specific to the development environment. These paths must be updated for a different installation before running the HLS tool flow.
