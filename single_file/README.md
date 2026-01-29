# SimTools v2.0 - 单文件版本

## 📦 这是什么？

这是 **SimTools v2.0 的单文件版本**，设计用于：
- ✅ 快速原型开发
- ✅ 学习 SimTools 库的使用
- ✅ 小型个人项目
- ✅ 不想配置复杂构建系统的场景

## 📁 文件结构

```
single_file/
├── SimTools_v2.h              # 主头文件（所有模块声明）
├── SimTools_v2_SingleFile.cpp  # 单文件实现（包含所有模块）
├── example.cpp                 # 示例程序
└── README.md                   # 本文件
```

**只需要这 3 个文件！** 🎉

---

## 🚀 快速开始

### 步骤 1：复制文件

将以下 3 个文件复制到你的项目目录：

1. `single_file/SimTools_v2.h`
2. `single_file/SimTools_v2_SingleFile.cpp`
3. `single_file/example.cpp`（可选，作为参考）

### 步骤 2：编写你的代码

创建 `main.cpp`：

```cpp
// 定义此标志以禁用 SimTools_v2_SingleFile.cpp 中的测试 main() 函数
#define SIMTOOLS_SINGLE_FILE_NO_MAIN

#include "SimTools_v2.h"
#include <iostream>

// 包含单文件实现（必须！）
#include "SimTools_v2_SingleFile.cpp"

using namespace SimTools;

int main() {
    // 现在可以使用所有功能了！
    auto air = Atmosphere::GetParameters(10000);
    std::cout << "温度: " << air.temperature << " K" << std::endl;

    auto pos = Coordinate::GpsToEcef({116.4, 39.9, 100});
    std::cout << "ECEF: " << pos << std::endl;

    return 0;
}
```

**注意**：
- `#define SIMTOOLS_SINGLE_FILE_NO_MAIN` 必须在 `#include "SimTools_v2_SingleFile.cpp"` 之前定义
- 这样可以避免与 SimTools_v2_SingleFile.cpp 中的测试 main() 函数冲突
- 如果你想直接运行 SimTools_v2_SingleFile.cpp 的测试程序，可以不定义这个标志，然后单独编译它

**直接运行测试程序**：

```bash
# 方法 1：编译 SimTools_v2_SingleFile.cpp（包含内置的测试 main()）
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC -DUSE_EIGEN \
    SimTools_v2_SingleFile.cpp -o test.exe
./test.exe

# 方法 2：编译 example.cpp（已经定义了 SIMTOOLS_SINGLE_FILE_NO_MAIN）
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC -DUSE_EIGEN \
    example.cpp -o example.exe
./example.exe
```

### 步骤 3：编译

**重要**: 由于 `main.cpp` 已经包含了 `SimTools_v2_SingleFile.cpp`，编译时**只需编译 main.cpp** 即可！

#### 方法 1：使用 g++ (MinGW/GCC)

```bash
# ✅ 正确：只编译 main.cpp
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    -o my_app.exe

# ❌ 错误：不要同时编译两个文件
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp SimTools_v2_SingleFile.cpp \
    -o my_app.exe  # 会导致符号重复定义错误
```

#### 方法 2：使用 Visual Studio 2019

1. 创建新的 C++ 控制台项目
2. 将 `SimTools_v2.h` 和 `SimTools_v2_SingleFile.cpp` 添加到项目
3. 配置项目属性：
   - **C/C++ → 常规 → 附加包含目录**：添加 `D:\eigen3`
   - **C/C++ → 预处理器 → 预处理器定义**：添加 `SIMTOOLS_STATIC;USE_EIGEN`
4. 编译运行

---

## 📖 功能示例

### 示例 1：坐标转换

```cpp
#include "SimTools_v2.h"
#include "SimTools_v2_SingleFile.cpp"
#include <iostream>

int main() {
    using namespace SimTools;

    // GPS 转 ECEF
    Vector3d gps(116.4, 39.9, 100);
    Vector3d ecef = Coordinate::GpsToEcef(gps);

    std::cout << "ECEF: " << ecef << std::endl;
    return 0;
}
```

### 示例 2：计算两点距离

```cpp
#include "SimTools_v2.h"
#include "SimTools_v2_SingleFile.cpp"
#include <iostream>

int main() {
    using namespace SimTools;

    // 北京到上海的距离
    double distance = Geodesy::GreatCircleDistance(
        116.4, 39.9,   // 北京
        121.5, 31.2    // 上海
    );

    std::cout << "距离: " << (distance / 1000.0) << " km" << std::endl;
    return 0;
}
```

### 示例 3：大气参数查询

```cpp
#include "SimTools_v2.h"
#include "SimTools_v2_SingleFile.cpp"
#include <iostream>

int main() {
    using namespace SimTools;

    // 查询 10km 高度的大气参数
    auto air = Atmosphere::GetParameters(10000);

    std::cout << "温度: " << air.temperature << " K" << std::endl;
    std::cout << "气压: " << air.pressure << " Pa" << std::endl;
    std::cout << "密度: " << air.density << " kg/m³" << std::endl;

    return 0;
}
```

---

## 🔧 编译命令

### 基础编译

```bash
# 只编译 main.cpp（因为它已经包含了 SimTools_v2_SingleFile.cpp）
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    -o app.exe
```

### 优化编译

```bash
# Release 优化
g++ -std=c++14 -O3 -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    -o app.exe

# 带调试信息
g++ -std=c++14 -g -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    -o app.exe
```

---

## 📋 可用功能列表

单文件版本包含**所有模块**的完整实现：

| 模块 | 功能 | 主要函数 |
|------|------|---------|
| **Math** | 数学工具 | Sign, Max, Min, Norm2, Regulate180, Normalize |
| **Interpolation** | 插值算法 | Linear, Lagrange7, CubicSpline, Bilinear |
| **Coordinate** | 坐标转换 | GpsToEcef, EcefToGps, EcefToNed |
| **Geodesy** | 地理计算 | GreatCircleDistance, Azimuth, Vincenty |
| **Atmosphere** | 大气模型 | GetParameters, Temperature, Density |
| **Random** | 随机数 | Uniform01, Normal, Exponential |
| **FileIO** | 文件I/O | ReadMatrix, WriteMatrix, CountLines |
| **Numerical** | 数值计算 | RungeKutta4, Bisection, Newton |
| **Geometry** | 几何计算 | IsPointInTriangle, DistanceToLineSegment |
| **MatrixUtils** | 矩阵工具 | EulerToMatrix, QuaternionToMatrix |
| **Time** | 时间工具 | UnixToGpsTime, GpsTimeToUnix |
| **Simulation** | 仿真实用工具 | Timer, Logger |

**所有功能都可以直接使用，无需额外配置！**

---

## ⚙️ 依赖项

### 必需

- **C++14 编译器** (GCC 5+, Clang 3.4+, MSVC 2015+)
- **Eigen3** (>= 3.3) - 矩阵运算库
  - Windows: `D:\eigen3`
  - Linux: `sudo apt-get install libeigen3-dev`
  - macOS: `brew install eigen`

### 可选

- CMake (如果不使用单文件版本)

---

## 🆚 与模块化版本的区别

| 特性 | 单文件版本 | 模块化版本 |
|------|-----------|-----------|
| 文件数量 | 2 个 (.h + .cpp) | 13 个 (.h + 12 个 .cpp) |
| 编译速度 | 每次完整编译 | 可增量编译 |
| 灵活性 | 包含所有功能 | 可选择性编译 |
| 适用场景 | 学习/原型/小项目 | 生产环境/大项目 |
| 推荐度 | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |

---

## 💡 使用技巧

### 技巧 1：缩短编译时间

如果经常修改代码，可以考虑：
- 使用预编译头文件
- 或者切换到模块化版本

### 技巧 2：调试模式

```bash
# 编译带调试信息的版本
g++ -std=c++14 -g -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    -o app_debug.exe

# 在 GDB 中调试
gdb app_debug.exe
```

### 技巧 3：优化性能

```bash
# 使用 Release 优化
g++ -std=c++14 -O3 -DNDEBUG \
    -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    -o app_fast.exe
```

---

## 🔍 常见问题

### Q1: 编译时提示找不到 Eigen3

**解决方案**：
```bash
# Windows
g++ -std=c++14 -I"D:\eigen3" -DSIMTOOLS_STATIC ...

# Linux
g++ -std=c++14 -I/usr/include/eigen3 -DSIMTOOLS_STATIC ...
```

### Q2: 符号重复定义错误

**解决方案**：由于 `main.cpp` 已经包含了 `SimTools_v2_SingleFile.cpp`，编译时**只编译 main.cpp**：

```bash
# 正确 ✅
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC main.cpp -o app

# 错误 ❌
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp SimTools_v2_SingleFile.cpp -o app  # 会导致符号重复定义
```

### Q3: 在 Visual Studio 中使用

**解决方案**：
1. 将 `SimTools_v2.h` 和 `SimTools_v2_SingleFile.cpp` 添加到项目
2. 设置 **C/C++ → 常规 → 附加包含目录**：`D:\eigen3`
3. 设置 **C/C++ → 预处理器 → 预处理器定义**：`SIMTOOLS_STATIC;USE_EIGEN`

---

## 📚 相关文档

- **上级目录**: `../README_SimTools.md` - 完整 API 文档
- **快速指南**: `../QUICKSTART_CN.md` - 快速入门
- **对比说明**: `../TWO_USAGE_MODES.md` - 两种版本详细对比

---

## 🎯 推荐工作流程

### 学习阶段

1. ✅ 使用本单文件版本
2. ✅ 运行 `example.cpp` 了解功能
3. ✅ 编写自己的小程序测试
4. ✅ 理解各个模块的功能

### 开发阶段

1. ✅ 使用单文件版本快速原型
2. ✅ 验证功能和性能
3. ⚠️ 项目长大后切换到模块化版本

### 生产阶段

1. ✅ 切换到模块化版本（上级目录）
2. ✅ 优化编译配置
3. ✅ 建立完整的构建系统

---

## 🎉 总结

单文件版本的 **SimTools_v2** 是一个：
- ✅ **简单**: 只需 2-3 个文件
- ✅ **完整**: 包含所有功能
- ✅ **独立**: 无需额外配置
- ✅ **快速**: 从零到运行只需几分钟

非常适合学习、原型开发和小型项目！

**开始使用 SimTools v2.0，让你的仿真开发更高效！** 🚀

---

**版本**: 2.0.0
**最后更新**: 2025-01-29
