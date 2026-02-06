# SimTools v2.0 - 快速开始

## 🚀 快速开始

### 方法 1：使用 Visual Studio 2019（推荐）

1. **双击运行** `open_vs2019.bat`
   - 这会自动生成并打开 VS2019 项目

2. **在 VS2019 中**：
   - 选择配置：**Release | x64**
   - 设置启动项目：右键 `SimTools_test` → 设为启动项目
   - 编译：按 `Ctrl+Shift+B`
   - 运行：按 `Ctrl+F5`

详细说明请查看：[VS2019_GUIDE.md](VS2019_GUIDE.md)

### 方法 2：使用命令行（MinGW/GCC）

```bash
# 编译并运行测试
cd E:\MyCode\SimTools_v2
g++ -std=c++14 -ID:\eigen3 -DSIMTOOLS_STATIC \
    SimTools_test.cpp \
    SimTools_*.cpp \
    -o SimTools_test.exe
./SimTools_test.exe
```

## 📁 项目结构

```
SimTools_v2/
├── SimTools_v2.h              # 主头文件（所有模块声明）
├── SimTools_*.cpp             # 模块实现文件（12个）
├── SimTools_test.cpp          # 单元测试
├── SimTools_v2_examples.cpp   # 示例代码
├── CMakeLists.txt             # CMake 构建脚本
├── open_vs2019.bat            # VS2019 快速启动脚本
├── build/                     # 构建目录（自动生成）
└── 文档/
    ├── README_SimTools.md     # 完整文档
    ├── QUICKSTART.md          # 快速入门
    ├── PROJECT_SUMMARY.md     # 项目概览
    └── VS2019_GUIDE.md        # VS2019 详细指南
```

## 🧪 运行测试

### Visual Studio 2019

1. 打开 `build/SimTools.sln`
2. 设置 `SimTools_test` 为启动项目
3. 按 `F5` 运行

### 命令行

```bash
# 运行测试程序
./build/Release/SimTools_test.exe

# 运行示例程序
./build/Release/SimTools_example.exe
```

## ✅ 测试结果

所有 11 个测试套件全部通过：

- ✅ Test_Math - 数学工具
- ✅ Test_Interpolation - 插值算法
- ✅ Test_Coordinate - 坐标转换
- ✅ Test_Geodesy - 地理计算
- ✅ Test_Atmosphere - 大气参数
- ✅ Test_Random - 随机数生成
- ✅ Test_Geometry - 几何计算
- ✅ Test_MatrixUtils - 矩阵工具
- ✅ Test_Units - 单位转换
- ✅ Test_Numerical - 数值计算
- ✅ Test_FileIO - 文件读写

## 📚 主要模块

| 模块 | 功能 | 应用场景 |
|------|------|---------|
| **Math** | 基础数学运算 | 向量运算、角度规范化 |
| **Interpolation** | 插值算法 | 数据插值、曲线拟合 |
| **Coordinate** | 坐标转换 | GPS ↔ ECEF ↔ NED |
| **Geodesy** | 地理计算 | 距离、方位角计算 |
| **Atmosphere** | 大气模型 | 温度、气压、密度 |
| **Random** | 随机数生成 | 统计模拟 |
| **Numerical** | 数值计算 | 常微分方程求解 |
| **Geometry** | 几何计算 | 点在多边形内、距离 |
| **MatrixUtils** | 矩阵工具 | 旋转矩阵、欧拉角 |
| **Time** | 时间工具 | GPS时间、Unix时间 |
| **Simulation** | 仿真实用工具 | 计时器、日志 |

## 🔧 依赖项

- **Eigen3** (>= 3.3) - 矩阵运算库
  - Windows: `D:\eigen3`
  - 或设置环境变量 `EIGEN_ROOT`

- **CMake** (>= 3.10) - 构建系统

- **C++14** 编译器
  - Visual Studio 2019+
  - GCC 5+
  - Clang 3.4+

## 🌟 特性

- ✅ 模块化设计
- ✅ 纯静态函数（无状态）
- ✅ 跨平台支持（Windows/Linux/macOS）
- ✅ 支持静态库和动态库
- ✅ 完整的单元测试
- ✅ 详细的文档和示例
- ✅ 高性能（GPS↔ECEF 转换 ~1μs）

## 📖 文档

- [README_SimTools.md](README_SimTools.md) - 完整 API 文档
- [QUICKSTART.md](QUICKSTART.md) - 快速入门
- [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) - 项目概览
- [VS2019_GUIDE.md](VS2019_GUIDE.md) - Visual Studio 2019 详细指南

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

MIT License

## 🔗 GitHub

https://github.com/ShadowMaverickSiri/MySimtools

---

**版本**: 2.0.0
**最后更新**: 2025-01-29
