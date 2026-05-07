# StarTracker V1

![Build Status](https://img.shields.io/badge/build-passing-brightgreen)
![C++](https://img.shields.io/badge/C%2B%2B-20-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green)

**StarTracker V1** is a multi-threaded C++ application for star tracking and orientation estimation using computer vision techniques. It reads image frames, identifies star centroids, calculates angular separations between detected stars, and maintains a lookup table for star pair matching. 

## 🚀 Features

- **Multi-threaded Architecture**: Independent reader (33 Hz) and processor (1 Hz) threads for non-blocking, asynchronous execution.
- **Robust Image Processing Pipeline**: Grayscale conversion, Gaussian blur, binary thresholding, and Connected Components Analysis.
- **Star Identification**: Converts 2D pixel coordinates to 3D normalized rays and computes precise angular separation.
- **Dynamic Terminal UI (TUI)**: Terminal-based interface using FTXUI (can be toggled via arguments).
- **Graceful Shutdown**: Safe thread joining and process termination via `SIGINT`.
- **Comprehensive Debugging**: Detailed logging levels and visual debug modes for centroids, rays, and angular separations.

## 🛠 Prerequisites

Ensure you have the following installed before building the project:
- **C++20** compatible compiler (g++ / clang++)
- **CMake** 3.10 or higher
- **OpenCV** (Required for vision processing)
- **pthread** (POSIX threads)

*(Note: `nlohmann_json` and `ftxui` are included as external subdirectories and compiled with the project).*

## ⚙️ Building the Project

The project uses CMake for its build system. To build StarTracker, follow these steps:

```bash
# Clone the repository
git clone <repository_url>
cd StarTracker

# Create a build directory
mkdir build && cd build

# Configure and compile
cmake ..
make
```

The resulting executable `startracker` will be placed in the `build/` directory.

## 🎮 Running the Project

Run the application directly from the build directory:

```bash
./startracker
```

### Command Line Arguments

You can pass several arguments to enable debug features and control logging verbosity:

| Argument | Description |
| --- | --- |
| `--tui` | Enables the Terminal User Interface (FTXUI). |
| `--debugc` | Enables centroid debug mode (outputs preprocessing details). |
| `--debugr` | Enables ray debug mode (outputs pixel and 3D ray vectors). |
| `--debuga` | Enables angular separation debug mode. |
| `--debugimg` | Opens OpenCV visualization windows (Original, Preprocessed, Centroids). |
| `--debugap` | Enables angular profile debug output. |
| `--log[0-4]` | Sets the logging verbosity level. (e.g., `--log0` for minimal, `--log4` for full debug info with source file locations). |

**Examples:**
```bash
# Run with TUI enabled
./startracker --tui

# Run with full debug visualizations and medium logging
./startracker --debugimg --debugr --log2

# Run with full text-based logging and debug modes
./startracker --debugc --debuga --log4
```

## 🧩 Project Structure

- `src/` - Core source code files (`main.cpp`, `processor.cpp`, `reader.cpp`, etc.).
- `include/` - Header files defining configurations, globals, and class structures.
- `external/` - Third-party libraries (`nlohmann_json`, `ftxui`).
- `config/` - JSON configuration files.
- `data/` - Contains pre-processed binary star catalog data (Hipparcos).
- `docs/` - Documentation and reference frames.
- `scripts/` - Auxiliary scripts.

## 📡 Pipeline Architecture

1. **Pre-processing**: The reader thread captures a frame (from a file or camera stream). The processor converts it to grayscale, applies a Gaussian blur, and uses a binary threshold to isolate bright objects.
2. **Star Detection**: Connected components are analyzed to filter noise (by pixel area) and extract centroids of true stars.
3. **3D Ray Projection**: The 2D pixel coordinates of centroids are transformed into 3D normalized rays in the camera body frame using the pinhole camera model.
4. **Angular Separation**: Computes the angular separation between all valid star pairs using vector dot products.
5. **Catalog Matching**: *(WIP)* Compares observed angular separation against the pre-loaded Hipparcos binary catalog to identify specific stars and estimate orientation.

---

## 📝 Notes on Real Trackers (Slew Rate / Blur Constraint)

*For reference on operational constraints:*

**Constraint:** Blur < 0.5 pixel (rule of thumb)

**Given:**
- Pixel scale = 0.1°/pixel
- Exposure = 50ms

**Calculation:**
Max allowed slew rate = `0.5 pixel × 0.1°/pixel ÷ 0.05s` = **1°/s**

**Handling High Slew Rates:**
If a satellite slews faster than 1°/s, the star tracker goes temporarily blind.
- IMU/gyro takes over attitude estimation during the maneuver.
- The tracker reacquires the star field after the slew settles.

*This is exactly why real satellites use sensor fusion — star tracker + gyroscope together, rather than relying on a star tracker alone.*