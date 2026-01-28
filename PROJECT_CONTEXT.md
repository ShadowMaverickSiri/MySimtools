# SimTools v2.0 - 项目上下文

## 📍 项目位置
- **本地路径**: `E:\MyCode\SimTools_v2`
- **相对路径**: `E:/MyCode/SimTools_v2`

## 🔗 Git 仓库
- **远程地址**: `git@github.com:ShadowMaverickSiri/MySimtools.git`
- **主分支**: `main`
- **最后提交**: `4fe4662 - Initial commit: SimTools v2.0`

## 📦 项目简介
专业的仿真工具库，为飞行器仿真、导弹制导、导航计算等应用提供工具函数。

**核心特性**:
- ✅ 纯静态函数库，无状态
- ✅ 12个功能模块
- ✅ 支持Eigen3或C++标准库
- ✅ 跨平台（Windows/Linux/macOS）
- ✅ 支持 DLL/SO 导出

## 📂 核心文件列表

### 头文件
- `SimTools_v2.h` (34KB) - 主头文件，所有模块声明

### 实现文件（12个模块）
- `SimTools_Math.cpp` - 数学工具
- `SimTools_Interpolation.cpp` - 插值算法
- `SimTools_Coordinate.cpp` - 坐标转换（GPS/ECEF/NED）
- `SimTools_Geodesy.cpp` - 地理计算（距离、方位角、Vincenty）
- `SimTools_Atmosphere.cpp` - 大气参数（温度、气压、密度、声速）
- `SimTools_Random.cpp` - 随机数生成
- `SimTools_FileIO.cpp` - 文件读写
- `SimTools_Numerical.cpp` - 数值计算（龙格-库塔、求根）
- `SimTools_Geometry.cpp` - 几何计算
- `SimTools_MatrixUtils.cpp` - 矩阵工具（四元数、欧拉角）
- `SimTools_Time.cpp` - 时间工具（GPS时间、Unix时间）
- `SimTools_Simulation.cpp` - 仿真实用工具（计时器、日志）

### 示例和测试
- `SimTools_v2_examples.cpp` (38KB) - 使用示例
- `SimTools_test.cpp` (9.6KB) - 单元测试

### 配置文件
- `CMakeLists.txt` - CMake 构建脚本（支持Eigen3自动检测）
- `.gitignore` - Git 忽略规则

### 文档
- `README.md` - 简易版主文档
- `README_SimTools.md` (14KB) - 完整技术文档
- `QUICKSTART.md` (6.4KB) - 快速入门指南
- `PROJECT_SUMMARY.md` (7.4KB) - 项目总览

## 🔧 技术栈

### 编译要求
- C++14 或更高
- CMake >= 3.10
- Eigen3 >= 3.3（可选）
  - 环境变量: `EIGEN_ROOT` = `D:/eigen3`
  - 如果没有Eigen3，自动使用C++标准库

### 编译命令
```bash
cd E:/MyCode/SimTools_v2
mkdir build && cd build
cmake .. -G "MinGW Makefiles"
mingw32-make -j4
```

## 🎯 下一步计划

- [ ] 修复编译错误（类型转换问题）
- [ ] 完善单元测试
- [ ] 添加 LICENSE 文件
- [ ] 性能优化
- [ ] 添加更多示例

## 📝 使用示例

```cpp
#include "SimTools_v2.h"
using namespace SimTools;

int main() {
    // 坐标转换
    Vector3d gps(116.4, 39.9, 100);
    auto ecef = Coordinate::GpsToEcef(gps);

    // 地理计算
    double dist = Geodesy::GreatCircleDistance(116.4, 39.9, 121.5, 31.2);

    // 大气参数
    auto air = Atmosphere::GetParameters(10000);

    return 0;
}
```

## 📧 联系方式
- GitHub: https://github.com/ShadowMaverickSiri/MySimtools
- Issues: https://github.com/ShadowMaverickSiri/MySimtools/issues

---
**最后更新**: 2025-01-23
**版本**: v2.0.0
