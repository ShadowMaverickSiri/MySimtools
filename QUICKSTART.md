# SimTools v2.0 快速入门指南

这是一个简单的入门指南，帮助你快速上手 SimTools 库。

## 📦 文件结构

编译完成后，你会得到以下文件：

```
SimTools/
├── SimTools_v2.h              # 主头文件（所有模块声明）
├── SimTools_*.cpp             # 实现文件
├── SimTools_v2_examples.cpp   # 示例程序
├── SimTools_test.cpp          # 单元测试
├── CMakeLists.txt             # CMake 构建脚本
└── README_SimTools.md         # 完整文档
```

## 🔨 编译步骤

### Windows (Visual Studio)

```cmd
# 1. 打开命令提示符，进入项目目录
cd E:\Code\Simtools

# 2. 创建构建目录
mkdir build
cd build

# 3. 生成 VS 解决方案
cmake .. -G "Visual Studio 16 2019"

# 4. 编译
cmake --build . --config Release

# 5. 运行示例
Release\SimTools_example.exe

# 6. 运行测试
Release\SimTools_test.exe
```

### Windows (MinGW)

```cmd
mkdir build
cd build
cmake .. -G "MinGW Makefiles"
cmake --build .
```

### Linux/macOS

```bash
mkdir build
cd build
cmake ..
make -j4

# 运行示例
./SimTools_example

# 运行测试
./SimTools_test
```

## 💡 基础用法

### 1. 包含头文件

```cpp
#include "SimTools_v2.h"

// 使用命名空间（可选，但推荐）
using namespace SimTools;
```

### 2. 坐标转换

```cpp
// GPS坐标：经度、纬度、高度
Coordinate::Vector3 beijing(116.3974, 39.9093, 100.0);

// 转换到 ECEF 坐标系
Coordinate::Vector3 ecef = Coordinate::GpsToEcef(beijing);

// 转换回 GPS
Coordinate::Vector3 gps_back = Coordinate::EcefToGps(ecef);
```

### 3. 地理计算

```cpp
// 计算两点间距离
double distance = Geodesy::GreatCircleDistance(
    116.4, 39.9,  // 北京
    121.5, 31.2   // 上海
);
std::cout << "距离: " << distance / 1000.0 << " km" << std::endl;

// 计算方位角
double azimuth = Geodesy::Azimuth(beijing, shanghai);
std::cout << "方位角: " << azimuth << " 度" << std::endl;
```

### 4. 大气参数

```cpp
// 获取 10km 高度的大气参数
auto air = Atmosphere::GetParameters(10000);

std::cout << "温度: " << air.temperature << " K" << std::endl;
std::cout << "气压: " << air.pressure << " Pa" << std::endl;
std::cout << "密度: " << air.density << " kg/m³" << std::endl;
std::cout << "声速: " << air.sound_speed << " m/s" << std::endl;
```

### 5. 插值计算

```cpp
// 准备数据
std::vector<double> x = {0, 1, 2, 3, 4, 5};
std::vector<double> y = {0, 1, 4, 9, 16, 25};  // y = x²

// 线性插值
double y_interp = Interpolation::Linear(2.5, x, y);

// 拉格朗日插值（更精确）
double y_lagrange = Interpolation::Lagrange7(2.5, x, y);
```

## 📝 完整示例

创建一个新文件 `my_first_simtools.cpp`：

```cpp
#include "SimTools_v2.h"
#include <iostream>

int main() {
    using namespace SimTools;

    std::cout << "=== SimTools v2.0 示例 ===" << std::endl << std::endl;

    // 1. 坐标转换
    std::cout << "1. 坐标转换:" << std::endl;
    Coordinate::Vector3 gps(116.4, 39.9, 100);
    auto ecef = Coordinate::GpsToEcef(gps);
    std::cout << "   GPS: " << gps.transpose() << std::endl;
    std::cout << "   ECEF: " << ecef.transpose() << std::endl << std::endl;

    // 2. 地理计算
    std::cout << "2. 地理计算:" << std::endl;
    double dist = Geodesy::GreatCircleDistance(116.4, 39.9, 121.5, 31.2);
    std::cout << "   北京-上海距离: " << dist / 1000.0 << " km" << std::endl << std::endl;

    // 3. 大气参数
    std::cout << "3. 大气参数 (10km):" << std::endl;
    auto air = Atmosphere::GetParameters(10000);
    std::cout << "   温度: " << air.temperature << " K" << std::endl;
    std::cout << "   声速: " << air.sound_speed << " m/s" << std::endl << std::endl;

    // 4. 插值
    std::cout << "4. 插值计算:" << std::endl;
    std::vector<double> x = {0, 1, 2, 3};
    std::vector<double> y = {0, 1, 4, 9};
    double y_interp = Interpolation::Linear(1.5, x, y);
    std::cout << "   f(1.5) = " << y_interp << std::endl << std::endl;

    // 5. 随机数
    std::cout << "5. 随机数:" << std::endl;
    Random::Seed(42);
    std::cout << "   均匀分布: " << Random::Uniform01() << std::endl;
    std::cout << "   正态分布: " << Random::Normal01() << std::endl << std::endl;

    return 0;
}
```

编译并运行：

```bash
g++ -std=c++14 -I/path/to/eigen3 -I. \
    my_first_simtools.cpp \
    SimTools_Interpolation.cpp \
    SimTools_Coordinate.cpp \
    SimTools_Geodesy.cpp \
    SimTools_Atmosphere.cpp \
    SimTools_Random.cpp \
    SimTools_MatrixUtils.cpp \
    SimTools_Simulation.cpp \
    -o my_app

./my_app
```

## 🔍 常见问题

### Q1: 找不到 Eigen3 库

**解决方法：**

1. 确保已安装 Eigen3
2. 设置环境变量或指定路径：

```bash
cmake .. -DEIGEN3_INCLUDE_DIR=/path/to/eigen3
```

### Q2: 编译错误 "undefined reference"

**解决方法：**

确保链接了所有需要的 `.cpp` 文件：

```bash
g++ main.cpp \
    SimTools_Interpolation.cpp \
    SimTools_Coordinate.cpp \
    SimTools_Geodesy.cpp \
    SimTools_Atmosphere.cpp \
    SimTools_Random.cpp \
    SimTools_FileIO.cpp \
    SimTools_Numerical.cpp \
    SimTools_Geometry.cpp \
    SimTools_MatrixUtils.cpp \
    SimTools_Time.cpp \
    SimTools_Simulation.cpp \
    -o my_app
```

### Q3: 使用 AFSIM 集成时如何配置？

**解决方法：**

编辑你的 AFSIM 项目 CMakeLists.txt：

```cmake
# 添加 SimTools 静态库
add_library(SimTools STATIC
    ${SIMTOOLS_DIR}/SimTools_*.cpp
)

target_include_directories(SimTools PUBLIC
    ${SIMTOOLS_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

# 链接到你的 AFSIM 插件
target_link_libraries(MyAFSIMPlugin PRIVATE SimTools)
```

## 📚 下一步

1. **查看完整示例** - 运行 `SimTools_example.exe`
2. **阅读 API 文档** - 查看 `README_SimTools.md`
3. **运行单元测试** - 运行 `SimTools_test.exe`
4. **查看源代码** - 阅读各个 `.cpp` 文件了解实现细节

## 🆘 获取帮助

- 查看完整文档：`README_SimTools.md`
- 查看示例代码：`SimTools_v2_examples.cpp`
- 运行单元测试：`SimTools_test.cpp`
- 提交问题：https://github.com/your-repo/SimTools/issues

祝你使用愉快！🎉
