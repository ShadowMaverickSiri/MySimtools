# SimTools v2.0

**专业的仿真工具库** - 为飞行器仿真、导弹制导、导航计算等应用提供丰富的数学、地理和物理计算工具。

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![C++](https://img.shields.io/badge/C++-14-blue.svg)]()
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey.svg)]()
[![Eigen](https://img.shields.io/badge/Eigen-3.3%2B-brightgreen.svg)]()

---

## ✨ 特性

- 🎯 **纯工具库** - 所有函数都是静态的，无状态，易于集成
- 📦 **模块化设计** - 12 个独立模块，职责清晰
- 🔌 **跨平台支持** - Windows/Linux/macOS，支持 DLL/SO 导出
- ⚡ **高性能** - 基于 Eigen3 库优化，提供高效的矩阵和四元数运算
- 📐 **完整的四元数支持** - 统一的四元数类，封装 Eigen 实现
- 📚 **完整文档** - 包含详细注释、示例代码和单元测试

---

## 📦 模块概览

| 模块 | 功能 |
|------|------|
| **Math** | 基础数学工具（符号、最大最小值、向量运算） |
| **Interpolation** | 插值算法（线性、拉格朗日、三次样条） |
| **Coordinate** | 坐标转换（GPS ↔ ECEF ↔ NED/NUE） |
| **Geodesy** | 地理计算（距离、方位角、Vincenty公式） |
| **Atmosphere** | 大气参数（温度、气压、密度、声速） |
| **Random** | 随机数生成（均匀分布、正态分布） |
| **FileIO** | 文件读写（CSV/TXT） |
| **Numerical** | 数值计算（龙格-库塔积分、求根） |
| **Geometry** | 几何计算（点在多边形内、距离） |
| **MatrixUtils** | 矩阵工具和四元数运算 |
| **Time** | 时间工具（GPS时间、Unix时间） |
| **Units** | 单位转换 |

---

## 🚀 快速开始

### 前置要求

- **C++ 编译器**: C++14 或更高版本
- **Eigen3**: 版本 3.3 或更高（必需）

#### 安装 Eigen3

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

或设置 `EIGEN_ROOT` 环境变量指向您的 Eigen 安装路径。

### 编译

```bash
# 克隆仓库
git clone https://github.com/ShadowMaverickSiri/MySimtools.git
cd MySimtools

# 编译
mkdir build && cd build
cmake ..
cmake --build .

# 运行测试
./SimTools_test
```

### 简单示例

```cpp
#include "SimTools_v2.h"
using namespace SimTools;

int main() {
    // 坐标转换
    Vector3d gps(116.4, 39.9, 100);  // 北京
    auto ecef = Coordinate::GpsToEcef(gps);

    // 地理计算
    double dist = Geodesy::GreatCircleDistance(116.4, 39.9, 121.5, 31.2);

    // 大气参数
    auto air = Atmosphere::GetParameters(10000);  // 10km
    std::cout << "声速: " << air.sound_speed << " m/s" << std::endl;

    // === 四元数姿态运算 ===
    // 从欧拉角创建四元数 (弧度)
    Quaterniond q = Quaterniond::FromEuler(0.1, 0.2, 0.3);

    // 四元数乘法（姿态组合）
    Quaterniond q2 = Quaterniond::FromAxisAngle(Vector3d(0, 0, 1), 0.5);
    Quaterniond q_combined = q * q2;

    // 旋转向量
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = q.Rotate(v);

    // SLERP 插值（姿态平滑过渡）
    Quaterniond q_interp = Quaterniond::Slerp(q, q2, 0.5);

    // 转换为欧拉角
    Vector3d euler = q.ToEuler();  // (roll, pitch, yaw)

    return 0;
}
```

### 四元数功能

`Quaternion` 类和 `MatrixUtils` 模块提供完整的四元数支持：

| 功能 | 函数 |
|------|------|
| 创建四元数 | `Quaterniond::Identity()`, `FromEuler()`, `FromAxisAngle()` |
| 四元数运算 | `operator*`, `Conjugate()`, `Inverse()`, `Dot()`, `Normalized()` |
| 旋转向量 | `q.Rotate(v)` 或 `MatrixUtils::RotateVector(q, v)` |
| 转换 | `ToEuler()`, `ToRotationMatrix()`, `ToAxisAngle()` |
| 插值 | `Quaterniond::Slerp(q0, q1, t)` |
| 微积分 | `Quaterniond::Exp()`, `Log()` |

---

## 🔧 最新更改

### 2025-03-30

- ✨ **四元数模块重构** - 现完全基于 Eigen 后端
- 🗑️ **移除旧代码** - 删除非 Eigen 版本的四元数实现
- 📝 **简化 API** - 统一的 `Quaternion` 类
- 🔧 **Eigen3 现为必需依赖** - 最低版本 3.3
- 📦 **移除冗余文件** - `SimTools_Quaternion.h` 已合并到主头文件
- 🎯 **更简洁的代码库** - 删除约 640 行冗余代码

---

## 📖 文档

- **快速入门**: [快速入门指南.md](快速入门指南.md)
- **API 参考**: [API完整参考.md](API完整参考.md)
- **单元测试**: [SimTools_test.cpp](SimTools_test.cpp)

---

## 🔧 依赖

### 必需依赖
- **CMake** (≥ 3.10)
- **C++14** 编译器 (GCC 5+, Clang 3.4+, MSVC 2015+)
- **Eigen3** (≥ 3.3) - 高性能矩阵和四元数运算库

---

## 📝 应用场景

- ✈️ 飞行器/导弹六自由度仿真
- 🛰️ 导航系统开发
- 📍 轨迹规划和目标定位
- 🌐 地理信息计算
- 📊 大气环境模拟
- 🔬 科学计算和工程仿真

---

## 🤝 贡献

欢迎贡献代码！请遵循以下步骤：

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

---

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 📧 联系方式

- **GitHub**: https://github.com/ShadowMaverickSiri/MySimtools
- **Issues**: https://github.com/ShadowMaverickSiri/MySimtools/issues

---

## 🌟 Star History

如果这个项目对你有帮助，请给个 ⭐️ Star！

---

**Made with ❤️ for the simulation community**
