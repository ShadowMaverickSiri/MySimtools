# SimTools v2.0

**Professional Simulation Tools Library** - Mathematical, geographical and physical computation tools for aircraft simulation, missile guidance, and navigation.

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![C++](https://img.shields.io/badge/C++-14-blue.svg)]()
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey.svg)]()
[![Eigen](https://img.shields.io/badge/Eigen-3.3%2B-brightgreen.svg)]()

---

## 📖 Documentation

**For complete documentation, please see:** [README_中文.md](README_中文.md)

### Quick Links

- 🚀 [Quick Start](快速入门指南.md) - Get started in 60 seconds
- 📚 [API Reference](API完整参考.md) - Complete function reference (120+ functions)
- 💻 [VS2019 Guide](VS2019_使用指南.md) - Visual Studio 2019 tutorial
- 📋 [Documentation Index](文档索引.md) - All documentation overview

---

## ✨ Features

- 🎯 **Pure Utility Library** - All static functions, stateless, easy to integrate
- 📦 **Modular Design** - 12 independent modules
- 🔌 **Cross-Platform** - Windows/Linux/macOS
- ⚡ **High Performance** - Powered by Eigen3 for optimized matrix and quaternion operations
- 📐 **Quaternion Support** - Complete quaternion math with Eigen backend
- 📚 **Complete Documentation** - Chinese documentation with examples

---

## 🚀 Quick Start

### Prerequisites

- **C++ Compiler**: C++14 or later
- **Eigen3**: Version 3.3 or higher (required)

#### Installing Eigen3

**Windows (vcpkg):**
```bash
vcpkg install eigen3:x64-windows
```

**Linux:**
```bash
sudo apt-get install libeigen3-dev
```

**macOS:**
```bash
brew install eigen
```

Or set `EIGEN_ROOT` environment variable to your Eigen installation path.

### Build

#### Windows (Visual Studio 2019)

```bash
# 1. Generate project
mkdir build && cd build
cmake -G "Visual Studio 16 2019" -A x64 ..

# 2. Open SimTools.sln in Visual Studio
# 3. Build and Run SimTools_test
```

#### Linux/macOS (GCC/Clang)

```bash
mkdir build && cd build
cmake ..
make
./SimTools_test
```

---

## 📦 Modules

| Module | Description |
|--------|-------------|
| **Math** | Basic math utilities (sign, clamp, angle regulation) |
| **Interpolation** | Linear, Lagrange, cubic spline interpolation |
| **Coordinate** | GPS ↔ ECEF ↔ NED coordinate conversions |
| **Geodesy** | Great circle, Haversine, Vincenty calculations |
| **Atmosphere** | ISA atmosphere model (pressure, density, temperature) |
| **Random** | Uniform and normal distribution generators |
| **FileIO** | File read/write utilities |
| **Numerical** | Runge-Kutta integration, root finding, derivatives |
| **Geometry** | Point-in-polygon, distance calculations |
| **Time** | GPS time, Unix time conversions |
| **Units** | Unit conversions (knots, feet, nautical miles, etc.) |
| **MatrixUtils** | Matrix operations and quaternion utilities |

---

## 🔧 Recent Changes

### 2025-03-30

- ✨ **Quaternion module refactored** - Now using pure Eigen backend
- 🗑️ **Removed legacy code** - Non-Eigen quaternion implementation removed
- 📝 **Simplified API** - Single, unified `Quaternion` class
- 🔧 **Eigen3 now required** - Minimum version 3.3
- 📦 **Removed redundant file** - `SimTools_Quaternion.h` merged into main header
- 🎯 **Cleaner codebase** - ~640 lines of redundant code removed

---

## 📋 Requirements

- **C++ Compiler**: C++14 or later
  - MSVC 19.20+ (Visual Studio 2019)
  - GCC 7.0+
  - Clang 5.0+
- **Required**: Eigen3 3.3+ (for matrix and quaternion operations)

---

## 📄 License

MIT License - see [LICENSE](LICENSE) for details

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

**中文用户请查看**: [README_中文.md](README_中文.md) for detailed documentation.
