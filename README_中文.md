# SimTools v2.0

**专业的仿真工具库** - 为飞行器仿真、导弹制导、导航计算等应用提供丰富的数学、地理和物理计算工具。

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![C++](https://img.shields.io/badge/C++-14-blue.svg)]()
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey.svg)]()

---

## ✨ 特性

- 🎯 **纯工具库** - 所有函数都是静态的，无状态，易于集成
- 📦 **模块化设计** - 12 个独立模块，职责清晰
- 🔌 **跨平台支持** - Windows/Linux/macOS，支持 DLL/SO 导出
- ⚡ **高性能** - 支持 Eigen3 库加速，也可使用纯 C++ 标准库
- 📚 **完整文档** - 包含详细注释、示例代码和单元测试
- 🛠️ **零依赖** - 如无 Eigen3，自动使用 C++ 标准库实现

---

## 📦 模块概览

| 模块 | 功能 |
|------|------|
| **Math** | 基础数学工具（符号、最大最小值、向量运算） |
| **Interpolation** | 插值算法（线性、拉格朗日、三次样条） |
| **Coordinate** | 坐标转换（GPS ↔ ECEF ↔ NED） |
| **Geodesy** | 地理计算（距离、方位角、Vincenty公式） |
| **Atmosphere** | 大气参数（温度、气压、密度、声速） |
| **Random** | 随机数生成（均匀分布、正态分布） |
| **FileIO** | 文件读写（CSV/TXT） |
| **Numerical** | 数值计算（龙格-库塔积分、求根） |
| **Geometry** | 几何计算（点在多边形内、距离） |
| **MatrixUtils** | 矩阵工具（四元数、欧拉角、旋转矩阵） |
| **Time** | 时间工具（GPS时间、Unix时间） |
| **Units** | 单位转换 |
| **Simulation** | 仿真实用工具（计时器、日志） |

---

## 🚀 快速开始

### 编译

```bash
# 克隆仓库
git clone https://github.com/ShadowMaverickSiri/MySimtools.git
cd MySimtools

# 编译
mkdir build && cd build
cmake ..
cmake --build .

# 运行示例
./SimTools_example
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
    Vector4d q = Vector4d::FromEuler(0.1, 0.2, 0.3);

    // 四元数乘法（姿态组合）
    Vector4d q2 = Vector4d::FromAxisAngle(Vector3d(0, 0, 1), 0.5);
    Vector4d q_combined = q * q2;

    // 旋转向量
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = q.rotate(v);

    // SLERP 插值（姿态平滑过渡）
    Vector4d q_interp = MatrixUtils::Slerp(q, q2, 0.5);

    // 转换为欧拉角
    Vector3d euler = q.toEuler();  // (roll, pitch, yaw)

    return 0;
}
```

### 四元数功能

`MatrixUtils` 模块现已提供完整的四元数支持：

| 功能 | 函数 |
|------|------|
| 创建四元数 | `Vector4d::Identity()`, `FromEuler()`, `FromAxisAngle()` |
| 四元数运算 | `operator*`, `conjugate()`, `inverse()`, `dot()`, `normalized()` |
| 旋转向量 | `q.rotate(v)` 或 `MatrixUtils::RotateVector(q, v)` |
| 转换 | `toEuler()`, `toAxisAngle()`, `MatrixToQuaternion()` |
| 插值 | `MatrixUtils::Slerp(q0, q1, t)` |
| 微积分 | `MatrixUtils::QuaternionExp()`, `QuaternionLog()` |

---

## 📖 文档

- **快速入门**: [QUICKSTART.md](QUICKSTART.md)
- **完整文档**: [README_SimTools.md](README_SimTools.md)
- **项目总览**: [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)
- **示例代码**: [SimTools_v2_examples.cpp](SimTools_v2_examples.cpp)
- **单元测试**: [SimTools_test.cpp](SimTools_test.cpp)

---

## 🔧 依赖

### 可选依赖
- **Eigen3** (≥ 3.3) - 高性能矩阵运算库（可选）
  - 如果没有安装，会自动使用 C++ 标准库实现

### 必需依赖
- **CMake** (≥ 3.10)
- **C++14** 编译器 (GCC 5+, Clang 3.4+, MSVC 2015+)

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
