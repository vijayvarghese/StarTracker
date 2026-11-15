# StarTracker V1

![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![CI](https://github.com/vijayvarghese/StarTracker/actions/workflows/ci.yml/badge.svg)
![Tests](https://img.shields.io/badge/tests-65%20passed-brightgreen)
![C++](https://img.shields.io/badge/C%2B%2B-20-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green)

**StarTracker V1** is a high-performance, multi-threaded C++20 application for spacecraft star tracking and autonomous attitude determination. It acquires star field frames (via file or TCP stream), extracts centroid coordinates using computer vision, computes 3D body rays, performs star identification using $N$-star voting against the Hipparcos catalog, and computes spacecraft orientation (ECI to body frame quaternion) using the **QUEST** algorithm with iterative **MAD** outlier rejection.

---

## 🚀 Key Features

- **QUEST Attitude Estimation**: Computes the optimal spacecraft attitude quaternion (ECI $\to$ body frame) using Davenport's K-matrix eigensolver (Shuster & Oh, 1981).
- **Iterative MAD Outlier Rejection**: Dynamically filters false-positive star identifications using Median Absolute Deviation of angular residuals across solver iterations.
- **$N$-Star Voting & Identification**: Fast angular distance binning against pre-processed Hipparcos catalog data to cast votes and identify stars with confidence scoring.
- **Multi-threaded Architecture**: Decoupled reader thread (33 Hz) and image processing thread (10 Hz) communicating via atomic shared pointers (`std::atomic<std::shared_ptr<cv::Mat>>`) for lock-free, zero-copy handoffs.
- **Hardware Abstraction Layer (HAL)**: Pluggable camera interfaces supporting static image sequences (`UnityMockFile`) and live TCP socket streams (`UnityMockTCP`) from Unity simulations.
- **Computer Vision Pipeline**: Grayscale conversion, Gaussian blurring with adaptive kernel validation, binary thresholding, and Connected Components Analysis with blob filtering.
- **Comprehensive Unit Tests (65 Tests)**: Fully automated GoogleTest and CTest test suites covering attitude, identification, image processing, geometry math, catalog loaders, config, and HAL.
- **Automated CI/CD**: GitHub Actions matrix workflow verifying builds and tests across GCC 13 and Clang 18 on Ubuntu 24.04.
- **Terminal User Interface (TUI)**: Optional FTXUI interactive terminal dashboard.

---

## 📡 Pipeline Architecture

```
[ Camera / TCP Stream ] ──(grabFrame)──> [ Reader Thread ]
                                                │ (atomic exchange)
                                                ▼
                                         [ latest_frame ]
                                                │
                                                ▼
                                     [ Preprocessing (OpenCV) ]
                                                │
                                                ▼
                                      [ Centroid Extraction ]
                                                │
                                                ▼
                                  [ Pinhole Ray Projection (b̂_i) ]
                                                │
                                                ▼
                                    [ Angular Separation (θ_ij) ]
                                                │
                                                ▼
                                     [ N-Star Voting Match ]
                                                │
                                                ▼
                                        [ Matched Stars ]
                                                │
                                                ▼
                              [ QUEST Solver + MAD Outlier Rejection ]
                                                │
                                                ▼
                           [ AttitudeResult: Quaternion (q), RMS, λ_max ]
```

---

## 🛠 Prerequisites

Ensure the following tools and libraries are installed:

- **C++20** compiler (`g++-13+` or `clang++-18+`)
- **CMake** (3.20 or higher)
- **OpenCV** 4.x (`libopencv-dev`)
- **POSIX Threads** (`pthread`)
- **Git**

*(Note: `Eigen3`, `nlohmann_json`, and `ftxui` are managed as Git submodules in `external/`).*

---

## ⚙️ Building the Project

### 1. Clone with Submodules
```bash
git clone --recurse-submodules https://github.com/vijayvarghese/StarTracker.git
cd StarTracker
```
*(If already cloned without `--recurse-submodules`, initialize them with `git submodule update --init --recursive`)*

### 2. Build Using CMake Presets (Recommended)

StarTracker includes predefined presets in `CMakePresets.json`:

| Preset | Purpose | Description |
| :--- | :--- | :--- |
| `dev` | Development | File HAL camera mode (`UnityMockFile`), Debug symbols |
| `dev-tcp` | Unity Simulation | TCP camera mode (`UnityMockTCP`), Debug symbols |
| `test` | Unit Tests | Enables GoogleTest suites and CTest runner |
| `tui` | Terminal UI | Builds with FTXUI dashboard interface |
| `devtools` | Utilities | Builds catalog generators (`Bin_Gen`, `Lookup_Gen`) |

```bash
# Configure and build dev target
cmake --preset dev
cmake --build --preset dev -j$(nproc)

# Configure and build TCP mode (for Unity simulator)
cmake --preset dev-tcp
cmake --build --preset dev-tcp -j$(nproc)
```

The compiled binary `startracker` will be placed in `build/<preset>/src/startracker`.

---

## 🧪 Running Unit Tests

The test suite covers all core algorithms with 65 unit tests via GoogleTest and CTest:

### Run All Tests via CTest
```bash
# Configure and build tests
cmake --preset test
cmake --build --preset test -j$(nproc)

# Run test suite
ctest --preset test --output-on-failure
```

### Run Individual Test Suites Directly
```bash
# Attitude determination & QUEST tests
./build/test/tests/test_attitude

# Star identification & N-star voting tests
./build/test/tests/test_identification

# Image processing & centroiding tests
./build/test/tests/test_image_proc

# Math & pinhole geometry tests
./build/test/tests/test_math

# Catalog loader & binary round-trip tests
./build/test/tests/test_catalog_loader

# Configuration tests
./build/test/tests/test_config

# Camera HAL tests
./build/test/tests/test_hal
```

---

## 🎮 Running the Application

Run the application directly:

```bash
./build/dev/src/startracker
```

### Command Line Arguments

Control debugging flags, logging levels, and UI features via CLI arguments:

| Argument | Description |
| :--- | :--- |
| `--q` | Enables QUEST solver debug output ($\lambda_{\max}$, inliers, RMS, quaternion). |
| `--debugc` | Enables centroid detection debug logging. |
| `--debugr` | Enables 3D body-ray projection debug logging. |
| `--debuga` | Enables pairwise angular separation debug logging. |
| `--debugap` | Enables candidate pair identification profile logging. |
| `--debugimg` | Displays live OpenCV preview windows (Preprocessed, Centroids, Frame). |
| `--tui` | Launches the interactive FTXUI Terminal User Interface (if built with `tui` preset). |
| `--log[0-4]` | Sets logging level (`--log0` minimal, `--log4` verbose debug with source locations). |

**Example:**
```bash
# Run with QUEST solver logs and visual inspection windows:
./build/dev/src/startracker --q --debugimg --log2
```

---

## 🧩 Project Structure

```
StarTracker/
├── .github/workflows/       # GitHub Actions CI workflow (GCC + Clang matrix)
├── config/                  # JSON configuration files (camera intrinsics, thresholds)
├── data/                    # Processed Hipparcos star catalog CSV & lookup bins
├── docs/                    # Documentation and coordinate frames
├── external/                # Submodules (Eigen, nlohmann_json, ftxui)
├── include/                 # Public C++ headers
│   └── startracker/
│       ├── attitude/        # QUEST attitude solver
│       ├── catalog/         # Catalog loader (CSV & binary formats)
│       ├── core/            # Types, thread-safe logger
│       ├── hal/             # Camera HAL (File & TCP mocks)
│       ├── identification/  # N-Star voting algorithm
│       ├── image/           # Preprocessing & centroiding
│       ├── math/            # Pinhole geometry & angular separation
│       └── util/            # Debug utilities
├── scripts/                 # Analysis and error visualization scripts (Python)
├── src/                     # Source implementation files
├── tests/                   # GoogleTest test suites (7 test suites, 65 tests)
├── CMakeLists.txt           # Root CMake build definition
└── CMakePresets.json        # CMake configuration and build presets
```

---

## 📝 Operational Notes: Slew Rate & Blur Constraints

*Real-world star tracker operational considerations:*

- **Constraint:** Motion blur $< 0.5\text{ pixel}$ across sensor exposure time.
- **Example:**
  - Pixel scale = $0.1^\circ/\text{pixel}$
  - Exposure = $50\text{ ms}$
  - $\text{Max slew rate} = \frac{0.5\text{ px} \times 0.1^\circ/\text{px}}{0.05\text{ s}} = \mathbf{1^\circ/\text{s}}$

If a spacecraft maneuvers faster than the optical slew limit, angular blur degrades centroid accuracy. During high-rate slews, spacecraft rely on gyroscope/IMU propagation and re-acquire absolute attitude with the star tracker once rates settle.

---

## 📄 License

This project is licensed under the MIT License.
