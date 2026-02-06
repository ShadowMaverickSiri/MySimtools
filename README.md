# SimTools v2.0

**Professional Simulation Tools Library** - Mathematical, geographical and physical computation tools for aircraft simulation, missile guidance, and navigation.

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![C++](https://img.shields.io/badge/C++-14-blue.svg)]()
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey.svg)]()

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
- 📦 **Modular Design** - 14 independent modules
- 🔌 **Cross-Platform** - Windows/Linux/macOS
- ⚡ **High Performance** - Eigen3 acceleration or pure C++ standard library
- 🛠️ **Zero Dependencies** - Auto-fallback to C++ standard library
- 📚 **Complete Documentation** - Chinese documentation with examples

---

## 🚀 Quick Start

### Windows (Visual Studio 2019)

```bash
# 1. Generate project
mkdir build && cd build
cmake -G "Visual Studio 16 2019" -A x64 ..

# 2. Open SimTools.sln in Visual Studio
# 3. Build and Run SimTools_test
```

### Linux/macOS (GCC/Clang)

```bash
# 1. Build
make

# 2. Run test
make run
```

---

## 📦 Modules

| Module | Description |
|--------|-------------|
| **Math** | Basic math utilities |
| **Interpolation** | Interpolation algorithms |
| **Coordinate** | GPS ↔ ECEF ↔ NED conversion |
| **Geodesy** | Geodesy calculations |
| **Atmosphere** | Atmosphere parameters |
| **Random** | Random number generation |
| **FileIO** | File I/O utilities |
| **Numerical** | Numerical computation |
| **Geometry** | Geometry calculations |
| **Time** | Time utilities |
| **Units** | Unit conversion |
| **MatrixUtils** | Matrix operations |
| **Simulation** | Simulation utilities |
| **Version** | Version information |

---

## 📋 Requirements

- **C++ Compiler**: C++14 or later
  - MSVC 19.20+ (Visual Studio 2019)
  - GCC 7.0+
  - Clang 5.0+
- **Optional**: Eigen3 3.3+ (for performance)

---

## 📄 License

MIT License - see [LICENSE](LICENSE) for details

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

**中文用户请查看**: [README_中文.md](README_中文.md) for detailed documentation.
