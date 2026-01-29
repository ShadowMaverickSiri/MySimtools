# SimTools v2.0 - 两种使用方式

## 📋 项目提供两种使用方式

SimTools v2.0 提供了**两种完全独立**的使用方式，你可以根据自己的需求选择：

---

## 🔄 方式对比

| 特性 | 方式 1：模块化 | 方式 2：单文件 |
|------|---------------|-------------|
| **文件** | SimTools_v2.h + 12 个 .cpp | SimTools_v2.h + SimTools_v2_SingleFile.cpp |
| **编译** | 需要链接所有 .cpp 文件 | 只需编译一个 .cpp 文件 |
| **灵活性** | ✅ 可选择性编译需要的模块 | ❌ 必须包含所有功能 |
| **依赖管理** | ✅ 清晰的模块边界 | ❌ 全部耦合在一起 |
| **适用场景** | 大型项目、部分使用功能 | 小型项目、快速原型 |
| **编译速度** | 增量编译快 | 每次都要完整编译 |
| **推荐度** | ⭐⭐⭐⭐⭐ 生产环境 | ⭐⭐⭐ 学习/原型 |

---

## 🎯 方式 1：模块化版本（推荐）

### 文件结构

```
SimTools_v2.h              # 主头文件（所有模块声明）
SimTools_Interpolation.cpp  # 插值模块
SimTools_Coordinate.cpp     # 坐标转换模块
SimTools_Geodesy.cpp        # 地理计算模块
SimTools_Atmosphere.cpp     # 大气参数模块
SimTools_Random.cpp         # 随机数模块
SimTools_FileIO.cpp         # 文件I/O模块
SimTools_Numerical.cpp       # 数值计算模块
SimTools_Geometry.cpp        # 几何计算模块
SimTools_MatrixUtils.cpp    # 矩阵工具模块
SimTools_Time.cpp           # 时间工具模块
SimTools_Simulation.cpp     # 仿真实用工具
```

### 使用方法

#### 方法 1：编译为静态库

```cmake
# CMakeLists.txt
add_library(SimTools_static STATIC
    SimTools_Interpolation.cpp
    SimTools_Coordinate.cpp
    # ... 其他 10 个 .cpp 文件
)

# 你的项目
target_link_libraries(MyApp PRIVATE SimTools_static)
```

#### 方法 2：直接编译

```bash
g++ -std=c++14 -I/path/to/eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    SimTools_Interpolation.cpp \
    SimTools_Coordinate.cpp \
    # ... 其他需要的模块
    -o my_app
```

### VS2019 使用

1. 打开 `build/SimTools.sln`
2. 选择 **Release | x64**
3. 编译整个解决方案
4. 链接到你的项目

---

## 🎯 方式 2：单文件版本

### 文件结构

```
SimTools_v2.h                    # 主头文件（所有模块声明）
SimTools_v2_SingleFile.cpp     # 单文件实现（包含所有模块）
```

### 使用方法

#### 编译

```bash
# 只需编译两个文件！
g++ -std=c++14 -I/path/to/eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    SimTools_v2_SingleFile.cpp \
    -o my_app
```

#### 代码示例

```cpp
#include "SimTools_v2.h"
#include <iostream>

// 只需包含这两个文件！
#include "SimTools_v2_SingleFile.cpp"

int main() {
    using namespace SimTools;

    // 直接使用所有功能
    auto air = Atmosphere::GetParameters(10000);
    std::cout << "温度: " << air.temperature << " K" << std::endl;

    auto pos = Coordinate::GpsToEcef({116.4, 39.9, 100});
    std::cout << "ECEF: " << pos << std::endl;

    return 0;
}
```

### 优势

✅ **简单**：只需复制两个文件到你的项目
✅ **快速**：不需要配置 CMake 或复杂的编译脚本
✅ **独立**：不依赖其他 .cpp 文件
✅ **学习**：适合学习库的使用

---

## 🎓 如何选择

### 选择模块化版本，如果你：

1. ✅ 开发大型项目
2. ✅ 只需要使用部分功能（节省编译时间）
3. ✅ 需要清晰的依赖管理
4. ✅ 团队协作开发
5. ✅ 使用 CMake 构建系统
6. ✅ **生产环境**（推荐）

### 选择单文件版本，如果你：

1. ✅ 快速原型开发
2. ✅ 学习 SimTools 库的使用
3. ✅ 小型个人项目
4. ✅ 不想配置复杂的构建系统
5. ✅ 只需要一个 .cpp 文件和 .h 文件
6. ✅ **测试/演示**（推荐）

---

## 📁 实际项目示例

### 示例 1：模块化项目（推荐）

```cpp
// main.cpp
#include "SimTools_v2.h"
#include <iostream>

// 只链接需要的模块
int main() {
    using namespace SimTools;

    // 只使用了 Coordinate 和 Atmosphere 模块
    auto air = Atmosphere::GetParameters(10000);
    auto pos = Coordinate::GpsToEcef({116.4, 39.9, 100});

    return 0;
}
```

**编译**：
```bash
g++ -std=c++14 -I/path/to/eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    SimTools_Coordinate.cpp \
    SimTools_Atmosphere.cpp \
    -o my_app
```

### 示例 2：单文件项目（快速）

```cpp
// main.cpp
#include "SimTools_v2.h"
#include <iostream>

// 包含单文件实现
#include "SimTools_v2_SingleFile.cpp"

int main() {
    using namespace SimTools;

    // 可以使用任何功能，无需单独链接
    auto air = Atmosphere::GetParameters(10000);
    auto pos = Coordinate::GpsToEcef({116.4, 39.9, 100});
    double dist = Geodesy::GreatCircleDistance(116.4, 39.9, 121.5, 31.2);

    return 0;
}
```

**编译**：
```bash
g++ -std=c++14 -I/path/to/eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    SimTools_v2_SingleFile.cpp \
    -o my_app
```

---

## 🆕 迁移指南

### 从单文件版本迁移到模块化版本

如果你先用单文件版本快速开发，后期需要迁移：

1. **保持代码不变**
   - 你的代码不需要修改！
   - `#include "SimTools_v2.h"` 保持不变

2. **删除单文件包含**
   - 移除 `#include "SimTools_v2_SingleFile.cpp"`

3. **添加需要的模块**
   - 在编译时链接需要的 .cpp 文件

4. **编译**
   ```bash
   g++ -std=c++14 -I/path/to/eigen3 -DSIMTOOLS_STATIC \
       main.cpp \
       SimTools_Coordinate.cpp \
       SimTools_Atmosphere.cpp \
       -o my_app
   ```

---

## 🎯 推荐实践

### 开发阶段

**使用单文件版本** 快速原型：
```cpp
#include "SimTools_v2.h"
#include "SimTools_v2_SingleFile.cpp"
```

### 生产阶段

**切换到模块化版本** 优化构建：
```cmake
add_executable(MyApp main.cpp)
target_link_libraries(MyApp PRIVATE SimTools_static)
```

---

## 📝 总结

| 场景 | 推荐方式 | 原因 |
|------|---------|------|
| **学习/测试** | 单文件 | 简单快速 |
| **原型开发** | 单文件 | 快速迭代 |
| **小型项目** | 模块化或单文件 | 都可以 |
| **大型项目** | 模块化 | 清晰的依赖 |
| **生产环境** | 模块化 | 增量编译、按需链接 |
| **团队协作** | 模块化 | 模块边界清晰 |

---

## 🔗 相关文档

- [QUICKSTART_CN.md](QUICKSTART_CN.md) - 快速开始指南
- [VS2019_GUIDE.md](VS2019_GUIDE.md) - Visual Studio 2019 详细指南
- [README_SimTools.md](README_SimTools.md) - 完整 API 文档

---

**Version**: 2.0.0
**Last Updated**: 2025-01-29
