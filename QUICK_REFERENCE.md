# SimTools v2.0 - 快速参考卡片

## 🚀 60秒快速开始

### 模块化版本（推荐生产环境）

```bash
# 编译
cd E:\MyCode\SimTools_v2
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    SimTools_Coordinate.cpp \
    SimTools_Atmosphere.cpp \
    -o my_app
```

### 单文件版本（推荐学习/原型）

```bash
# 编译
cd E:\MyCode\SimTools_v2
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    SimTools_v2_SingleFile.cpp \
    -o my_app
```

---

## 📋 文件清单

### 方式 1：模块化

| 文件 | 必需？ | 说明 |
|------|--------|------|
| `SimTools_v2.h` | ✅ 必需 | 主头文件（所有模块声明）|
| `SimTools_Coordinate.cpp` | ⚠️ 按需 | 坐标转换 |
| `SimTools_Atmosphere.cpp` | ⚠️ 按需 | 大气参数 |
| `SimTools_Geodesy.cpp` | ⚠️ 按需 | 地理计算 |
| ... | ⚠️ 按需 | 其他模块 |

**依赖关系**：所有 .cpp 文件都依赖 `SimTools_v2.h`

### 方式 2：单文件

| 文件 | 必需？ | 说明 |
|------|--------|------|
| `SimTools_v2.h` | ✅ 必需 | 主头文件（所有模块声明）|
| `SimTools_v2_SingleFile.cpp` | ✅ 必需 | 单文件实现（所有模块）|

**依赖关系**：只需这两个文件！

---

## 🎯 使用场景速查表

```
你的需求是...
├─ 学习 SimTools 库
│  └─→ 使用单文件版本 ✅
│
├─ 快速原型开发
│  └─→ 使用单文件版本 ✅
│
├─ 小型个人项目
│  └─→ 单文件版本或模块化版本均可 ✅
│
├─ 大型项目
│  └─→ 使用模块化版本 ✅
│
├─ 团队协作开发
│  └─→ 使用模块化版本 ✅
│
├─ 生产环境
│  └─→ 使用模块化版本 ✅
│
└─ 只需要部分功能
   └─→ 使用模块化版本 ✅（按需链接）
```

---

## 💻 代码示例

### 方式 1：模块化

```cpp
// main.cpp
#include "SimTools_v2.h"
#include <iostream>

int main() {
    using namespace SimTools;

    // 可用功能
    auto air = Atmosphere::GetParameters(10000);
    auto pos = Coordinate::GpsToEcef({116.4, 39.9, 100});

    return 0;
}
```

**编译**：
```bash
# 只链接需要的模块
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    SimTools_Coordinate.cpp \
    SimTools_Atmosphere.cpp \
    -o app
```

### 方式 2：单文件

```cpp
// main.cpp
#include "SimTools_v2.h"
#include <iostream>

// 包含单文件实现！
#include "SimTools_v2_SingleFile.cpp"

int main() {
    using namespace SimTools;

    // 所有功能都可用
    auto air = Atmosphere::GetParameters(10000);
    auto pos = Coordinate::GpsToEcef({116.4, 39.9, 100});
    double dist = Geodesy::GreatCircleDistance(116.4, 39.9, 121.5, 31.2);

    return 0;
}
```

**编译**：
```bash
# 只需这两个文件！
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    SimTools_v2_SingleFile.cpp \
    -o app
```

---

## 🔧 Visual Studio 2019

### 方式 1：模块化（已在 build/SimTools.sln 中）

1. 打开 `build/SimTools.sln`
2. 选择 **Release | x64**
3. 编译整个解决方案
4. 在你的项目中链接 `SimTools_static.lib`

### 方式 2：单文件

1. 创建新的 C++ 项目
2. 添加 `SimTools_v2.h` 和 `SimTools_v2_SingleFile.cpp`
3. 设置 Eigen3 路径：`D:\eigen3`
4. 添加预处理器定义：`SIMTOOLS_STATIC;USE_EIGEN`
5. 编译运行

---

## ✅ 功能验证

运行后应该看到：

```
========================================
  SimTools v2.0 - 功能演示
========================================
Version: 2.0.0

1. 坐标转换测试...
2. 地理计算测试...
3. 大气参数测试...
...
========================================
  演示完成！
========================================
```

---

## 📚 完整文档

| 文档 | 内容 |
|------|------|
| **TWO_USAGE_MODES.md** | 两种使用方式详细说明 |
| **QUICKSTART_CN.md** | 快速开始指南 |
| **VS2019_GUIDE.md** | VS2019 详细指南 |
| **README_SimTools.md** | 完整 API 文档 |

---

## 🎓 推荐学习路径

1. **入门阶段**：使用单文件版本快速学习
2. **项目开发**：根据项目规模选择合适的版本
3. **生产部署**：使用模块化版本优化

---

## 🆘 快速切换

### 从单文件切换到模块化

```bash
# 原来（单文件）
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp SimTools_v2_SingleFile.cpp -o app

# 改为（模块化）
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC \
    main.cpp \
    SimTools_Coordinate.cpp \
    SimTools_Atmosphere.cpp \
    -o app
```

你的代码 **不需要修改**！

---

**选择适合你的方式，开始使用 SimTools v2.0！** 🚀
