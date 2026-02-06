# SimTools v2.0 - 函数功能说明

本文档详细说明 SimTools v2.0 库中所有功能函数的用途、输入输出格式及使用方法。

---

## 目录

1. [数学工具模块 (Math)](#1-数学工具模块-math)
2. [插值算法模块 (Interpolation)](#2-插值算法模块-interpolation)
3. [坐标转换模块 (Coordinate)](#3-坐标转换模块-coordinate)
4. [地理计算模块 (Geodesy)](#4-地理计算模块-geodesy)
5. [大气参数模块 (Atmosphere)](#5-大气参数模块-atmosphere)
6. [随机数生成模块 (Random)](#6-随机数生成模块-random)
7. [文件I/O模块 (FileIO)](#7-文件io模块-fileio)
8. [数值计算模块 (Numerical)](#8-数值计算模块-numerical)
9. [几何计算模块 (Geometry)](#9-几何计算模块-geometry)
10. [时间工具模块 (Time)](#10-时间工具模块-time)
11. [单位转换模块 (Units)](#11-单位转换模块-units)
12. [矩阵运算模块 (MatrixUtils)](#12-矩阵运算模块-matrixutils)
13. [仿真实用工具模块 (Simulation)](#13-仿真实用工具模块-simulation)
14. [版本信息模块 (Version)](#14-版本信息模块-version)

---

## 1. 数学工具模块 (Math)

### 1.1 Sign
- **功能**：返回数值的符号
- **输入**：`T x` - 任意数值
- **输出**：`T` - 返回 -1（负数）、0（零）、1（正数）
- **用法**：`Math::Sign(-5.0)` 返回 -1

### 1.2 Max
- **功能**：返回两个数中的最大值
- **输入**：`T a, T b` - 两个数值
- **输出**：`T` - 较大的值
- **用法**：`Math::Max(3.0, 7.0)` 返回 7.0

### 1.3 Min
- **功能**：返回两个数中的最小值
- **输入**：`T a, T b` - 两个数值
- **输出**：`T` - 较小的值
- **用法**：`Math::Min(3.0, 7.0)` 返回 3.0

### 1.4 Clamp
- **功能**：将数值限制在指定范围内
- **输入**：`T value, T min_val, T max_val` - 值、最小值、最大值
- **输出**：`T` - 限制后的值
- **用法**：`Math::Clamp(5.0, 0.0, 10.0)` 返回 5.0

### 1.5 FindAbsMax
- **功能**：查找数组中绝对值最大的元素及其索引
- **输入**：`const std::vector<T>& data, int& index` - 数据数组、索引引用
- **输出**：`T` - 绝对值最大的元素值，索引通过引用返回
- **用法**：`double max = Math::FindAbsMax(data, idx);`

### 1.6 Norm2 (Vector3d)
- **功能**：计算3D向量的二范数（长度）
- **输入**：`const Vector3d& vec` - 3D向量
- **输出**：`double` - 向量长度
- **用法**：`double len = Math::Norm2(vector);`

### 1.7 Norm2 (VectorXd)
- **功能**：计算动态向量的二范数
- **输入**：`const VectorXd& vec` - 动态向量
- **输出**：`double` - 向量长度
- **用法**：`double len = Math::Norm2(vector);`

### 1.8 Normalize
- **功能**：将3D向量归一化（单位化）
- **输入**：`const Vector3d& vec` - 3D向量
- **输出**：`Vector3d` - 单位向量
- **用法**：`Vector3d unit = Math::Normalize(vector);`

### 1.9 Regulate180
- **功能**：将角度规范化到 [-180°, 180°] 范围
- **输入**：`double angle` - 角度（度）
- **输出**：`double` - 规范化后的角度
- **用法**：`Math::Regulate180(370.0)` 返回 10.0

### 1.10 Regulate360
- **功能**：将角度规范化到 [0°, 360°] 范围
- **输入**：`double angle` - 角度（度）
- **输出**：`double` - 规范化后的角度
- **用法**：`Math::Regulate360(-10.0)` 返回 350.0

### 1.11 RegulatePi
- **功能**：将弧度规范化到 [-π, π] 范围
- **输入**：`double angle` - 弧度
- **输出**：`double` - 规范化后的弧度
- **用法**：`Math::RegulatePi(4.0)` 返回约 0.7168

---

## 2. 插值算法模块 (Interpolation)

### 2.1 Linear
- **功能**：一维线性插值
- **输入**：`double x, const std::vector<double>& xx, const std::vector<double>& yy` - 查询点、x数组、y数组
- **输出**：`double` - 插值结果
- **用法**：`double y = Interpolation::Linear(2.5, x_data, y_data);`

### 2.2 Lagrange7
- **功能**：使用前后7个点进行拉格朗日插值
- **输入**：`double x, const std::vector<double>& xx, const std::vector<double>& yy`
- **输出**：`double` - 插值结果
- **用法**：`double y = Interpolation::Lagrange7(2.5, x_data, y_data);`

### 2.3 LagrangeGlobal
- **功能**：全局拉格朗日插值（使用所有点）
- **输入**：`double x, const std::vector<double>& xx, const std::vector<double>& yy`
- **输出**：`double` - 插值结果
- **用法**：`double y = Interpolation::LagrangeGlobal(2.5, x_data, y_data);`

### 2.4 CubicSpline
- **功能**：三次样条插值
- **输入**：`double x, const std::vector<double>& xx, const std::vector<double>& yy`
- **输出**：`double` - 插值结果
- **用法**：`double y = Interpolation::CubicSpline(2.5, x_data, y_data);`

### 2.5 Bilinear
- **功能**：双线性二维插值
- **输入**：`double u, double v, const std::vector<double>& x, const std::vector<double>& y, const std::vector<std::vector<double>>& z`
- **输出**：`double` - 插值结果
- **用法**：`double z = Interpolation::Bilinear(u, v, x, y, z_data);`

### 2.6 FindIndex
- **功能**：查找x在xx数组中的插入位置索引
- **输入**：`double x, const std::vector<double>& xx`
- **输出**：`int` - 索引位置
- **用法**：`int idx = Interpolation::FindIndex(2.5, x_data);`

### 2.7 FindIndex2D
- **功能**：查找二维插值的索引位置
- **输入**：`double u, double v, const std::vector<double>& x, const std::vector<double>& y`
- **输出**：`std::pair<int, int>` - (i, j) 索引对
- **用法**：`auto idx = Interpolation::FindIndex2D(u, v, x, y);`

---

## 3. 坐标转换模块 (Coordinate)

### 3.1 GpsToEcef
- **功能**：GPS坐标转换为地心地固坐标系(ECEF)
- **输入**：`const Vector3& gps` - (经度°, 纬度°, 高度m)
- **输出**：`Vector3` - ECEF坐标 (X, Y, Z)m
- **用法**：`Vector3 ecef = Coordinate::GpsToEcef(gps);`

### 3.2 EcefToGps
- **功能**：ECEF坐标转换为GPS坐标
- **输入**：`const Vector3& ecef` - ECEF坐标 (X, Y, Z)m
- **输出**：`Vector3` - GPS坐标 (经度°, 纬度°, 高度m)
- **用法**：`Vector3 gps = Coordinate::EcefToGps(ecef);`

### 3.3 EcefToGpsNewton
- **功能**：ECEF坐标转换为GPS坐标（使用牛顿迭代，精度更高）
- **输入**：`const Vector3& ecef`
- **输出**：`Vector3` - GPS坐标
- **用法**：`Vector3 gps = Coordinate::EcefToGpsNewton(ecef);`

### 3.4 EcefToNedMatrix
- **功能**：创建ECEF到NED的旋转矩阵
- **输入**：`double longitude, double latitude` - 经度、纬度
- **输出**：`Matrix3` - 旋转矩阵
- **用法**：`Matrix3 R = Coordinate::EcefToNedMatrix(lon, lat);`

### 3.5 NedToEcefMatrix
- **功能**：创建NED到ECEF的旋转矩阵
- **输入**：`double longitude, double latitude`
- **输出**：`Matrix3` - 旋转矩阵
- **用法**：`Matrix3 R = Coordinate::NedToEcefMatrix(lon, lat);`

### 3.6 EcefToNed
- **功能**：ECEF位置转换为NED位置（相对于参考点）
- **输入**：`const Vector3& ecef_pos, const Vector3& ref_gps` - ECEF位置、参考点GPS
- **输出**：`Vector3` - NED位置 (北, 东, 地)m
- **用法**：`Vector3 ned = Coordinate::EcefToNed(ecef, ref_gps);`

### 3.7 NedToEcef
- **功能**：NED位置转换为ECEF位置
- **输入**：`const Vector3& ned_pos, const Vector3& ref_gps`
- **输出**：`Vector3` - ECEF位置
- **用法**：`Vector3 ecef = Coordinate::NedToEcef(ned, ref_gps);`

### 3.8 VelocityToNed
- **功能**：弹道坐标系速度转换为NED速度
- **输入**：`double theta, double phi_v, double V, Vector3& vn` - 弹道倾斜角、弹道偏角、速度、输出速度引用
- **输出**：通过引用返回NED速度
- **用法**：`Coordinate::VelocityToNed(theta, phi_v, V, ned_vel);`

### 3.9 NedToEcefVelocity
- **功能**：NED速度转换为ECEF速度
- **输入**：`const Vector3& ned_vel, const Vector3& gps_pos, Vector3& ecef_vel`
- **输出**：通过引用返回ECEF速度
- **用法**：`Coordinate::NedToEcefVelocity(ned_vel, gps, ecef_vel);`

### 3.10 RotationMatrix
- **功能**：创建基本旋转矩阵
- **输入**：`double angle, int axis` - 角度、轴(1=X, 2=Y, 3=Z)
- **输出**：`Matrix3` - 旋转矩阵
- **用法**：`Matrix3 R = Coordinate::RotationMatrix(angle, 3);`

### 3.11 DmsToDecimal
- **功能**：度分秒转换为十进制
- **输入**：`double degree, double minute, double second`
- **输出**：`double` - 十进制角度
- **用法**：`double dec = Coordinate::DmsToDecimal(120, 30, 36);`

### 3.12 DecimalToDms
- **功能**：十进制转换为度分秒
- **输入**：`double decimal, int& degree, int& minute, double& second`
- **输出**：通过引用返回度分秒
- **用法**：`Coordinate::DecimalToDms(120.51, deg, min, sec);`

### 3.13 RadToDeg
- **功能**：弧度转角度
- **输入**：`double rad`
- **输出**：`double` - 角度
- **用法**：`double deg = Coordinate::RadToDeg(3.14159);`

### 3.14 DegToRad
- **功能**：角度转弧度
- **输入**：`double deg`
- **输出**：`double` - 弧度
- **用法**：`double rad = Coordinate::DegToRad(180.0);`

---

## 4. 地理计算模块 (Geodesy)

### 4.1 GreatCircleDistance
- **功能**：计算两点间的大圆距离（球面模型）
- **输入**：`double lon1, double lat1, double lon2, double lat2` - 两点的经纬度
- **输出**：`double` - 距离（米）
- **用法**：`double dist = Geodesy::GreatCircleDistance(lon1, lat1, lon2, lat2);`

### 4.2 HaversineDistance
- **功能**：使用Haversine公式计算距离（球面模型）
- **输入**：`double lon1, double lat1, double lon2, double lat2`
- **输出**：`double` - 距离（米）
- **用法**：`double dist = Geodesy::HaversineDistance(lon1, lat1, lon2, lat2);`

### 4.3 VincentyDistance
- **功能**：计算Vincenty距离（椭球面模型，高精度）
- **输入**：`double lon1, double lat1, double lon2, double lat2`
- **输出**：`double` - 距离（米）
- **用法**：`double dist = Geodesy::VincentyDistance(lon1, lat1, lon2, lat2);`

### 4.4 Azimuth
- **功能**：计算初始方位角（从点1到点2）
- **输入**：`const Vector3d& gps1, const Vector3d& gps2` - 两点GPS坐标
- **输出**：`double` - 方位角（度）
- **用法**：`double az = Geodesy::Azimuth(gps1, gps2);`

### 4.5 AzimuthAndDistance
- **功能**：计算两点间的方位角和距离
- **输入**：`const Vector3d& gps1, const Vector3d& gps2, double& azimuth, double& distance`
- **输出**：通过引用返回方位角和距离
- **用法**：`Geodesy::AzimuthAndDistance(gps1, gps2, az, dist);`

### 4.6 VincentyInverse
- **功能**：Vincenty反解（已知两点求距离和方位角）
- **输入**：`double lon1, double lat1, double lon2, double lat2, double& distance, double& azimuth1, double& azimuth2`
- **输出**：通过引用返回距离和两个方位角
- **用法**：`Geodesy::VincentyInverse(lon1, lat1, lon2, lat2, dist, az1, az2);`

### 4.7 VincentyDirect
- **功能**：Vincenty正解（已知起点、方位角和距离求终点）
- **输入**：`double lon1, double lat1, double azimuth, double distance, double& lon2, double& lat2, double& azimuth2`
- **输出**：通过引用返回终点坐标和反方位角
- **用法**：`Geodesy::VincentyDirect(lon1, lat1, az, dist, lon2, lat2, az2);`

### 4.8 TargetFromSite
- **功能**：根据观测站、方位角、高度和斜距计算目标GPS坐标
- **输入**：`const Vector3d& site_gps, double azimuth, double target_height, double slant_range`
- **输出**：`Vector3d` - 目标GPS坐标
- **用法**：`Vector3d target = Geodesy::TargetFromSite(site, az, h, range);`

### 4.9 SiteAzimuth
- **功能**：计算两GPS点间的站心方位角
- **输入**：`const Vector3d& gpsA, const Vector3d& gpsB`
- **输出**：`double` - 方位角（度）
- **用法**：`double az = Geodesy::SiteAzimuth(gpsA, gpsB);`

### 4.10 SiteDistance
- **功能**：计算两GPS点间的站心斜距
- **输入**：`const Vector3d& gpsA, const Vector3d& gpsB`
- **输出**：`double` - 距离（米）
- **用法**：`double dist = Geodesy::SiteDistance(gpsA, gpsB);`

---

## 5. 大气参数模块 (Atmosphere)

### 5.1 GetParameters
- **功能**：获取指定高度的完整大气参数
- **输入**：`double height_meters` - 高度（米）
- **输出**：`Parameters` - 包含气压、密度、重力、声速、温度
- **用法**：`auto params = Atmosphere::GetParameters(10000.0);`

### 5.2 Gravity
- **功能**：计算重力加速度
- **输入**：`double height_meters`
- **输出**：`double` - 重力加速度 (m/s²)
- **用法**：`double g = Atmosphere::Gravity(10000.0);`

### 5.3 SoundSpeed
- **功能**：计算声速
- **输入**：`double height_meters`
- **输出**：`double` - 声速 (m/s)
- **用法**：`double vs = Atmosphere::SoundSpeed(10000.0);`

### 5.4 Density
- **功能**：计算空气密度
- **输入**：`double height_meters`
- **输出**：`double` - 空气密度 (kg/m³)
- **用法**：`double rho = Atmosphere::Density(10000.0);`

### 5.5 Pressure
- **功能**：计算气压
- **输入**：`double height_meters`
- **输出**：`double` - 气压 (Pa)
- **用法**：`double p = Atmosphere::Pressure(10000.0);`

### 5.6 Temperature
- **功能**：计算温度
- **输入**：`double height_meters`
- **输出**：`double` - 温度 (K)
- **用法**：`double T = Atmosphere::Temperature(10000.0);`

### 5.7 VelocityFromMach
- **功能**：根据马赫数和高度计算速度
- **输入**：`double mach, double height_meters`
- **输出**：`double` - 速度 (m/s)
- **用法**：`double v = Atmosphere::VelocityFromMach(2.0, 10000.0);`

### 5.8 MachFromVelocity
- **功能**：根据速度和高度计算马赫数
- **输入**：`double velocity, double height_meters`
- **输出**：`double` - 马赫数
- **用法**：`double mach = Atmosphere::MachFromVelocity(680.0, 10000.0);`

### 5.9 DynamicPressure
- **功能**：计算动压 q = 0.5 * rho * V²
- **输入**：`double velocity, double height_meters`
- **输出**：`double` - 动压 (Pa)
- **用法**：`double q = Atmosphere::DynamicPressure(300.0, 0.0);`

---

## 6. 随机数生成模块 (Random)

### 6.1 Seed
- **功能**：设置随机种子
- **输入**：`unsigned int seed` - 种子值（0表示使用当前时间）
- **输出**：无
- **用法**：`Random::Seed(42);`

### 6.2 Uniform01
- **功能**：生成[0, 1)均匀分布随机数
- **输入**：无
- **输出**：`double` - 随机数
- **用法**：`double r = Random::Uniform01();`

### 6.3 Uniform
- **功能**：生成[a, b]均匀分布随机数
- **输入**：`double a, double b` - 区间端点
- **输出**：`double` - 随机数
- **用法**：`double r = Random::Uniform(10.0, 20.0);`

### 6.4 UniformInt
- **功能**：生成整数均匀分布
- **输入**：`int a, int b` - 区间端点
- **输出**：`int` - 随机整数
- **用法**：`int r = Random::UniformInt(1, 100);`

### 6.5 Normal01
- **功能**：生成标准正态分布N(0, 1)
- **输入**：无
- **输出**：`double` - 随机数
- **用法**：`double r = Random::Normal01();`

### 6.6 Normal
- **功能**：生成正态分布N(mu, sigma²)
- **输入**：`double mu, double sigma` - 均值、标准差
- **输出**：`double` - 随机数
- **用法**：`double r = Random::Normal(100.0, 15.0);`

### 6.7 Exponential
- **功能**：生成指数分布
- **输入**：`double lambda` - 参数
- **输出**：`double` - 随机数
- **用法**：`double r = Random::Exponential(2.0);`

### 6.8 Weibull
- **功能**：生成韦伯分布
- **输入**：`double shape, double scale` - 形状参数、尺度参数
- **输出**：`double` - 随机数
- **用法**：`double r = Random::Weibull(2.0, 1.0);`

---

## 7. 文件I/O模块 (FileIO)

### 7.1 ReadMatrix
- **功能**：读取文本文件到矩阵
- **输入**：`const std::string& filename` - 文件名
- **输出**：`MatrixXd` - 数据矩阵
- **用法**：`MatrixXd mat = FileIO::ReadMatrix("data.txt");`

### 7.2 ReadTable
- **功能**：读取文本文件到二维vector
- **输入**：`const std::string& filename`
- **输出**：`std::vector<std::vector<double>>` - 数据表
- **用法**：`auto table = FileIO::ReadTable("data.txt");`

### 7.3 ReadColumn
- **功能**：读取单列数据
- **输入**：`const std::string& filename, int column_index` - 文件名、列索引（默认0）
- **输出**：`std::vector<double>` - 数据列
- **用法**：`vector<double> col = FileIO::ReadColumn("data.txt", 0);`

### 7.4 WriteMatrix
- **功能**：将矩阵写入文件
- **输入**：`const std::string& filename, const MatrixXd& matrix, bool scientific, int precision`
- **输出**：`bool` - 成功返回true
- **用法**：`FileIO::WriteMatrix("out.txt", matrix, false, 6);`

### 7.5 WriteVector
- **功能**：将vector写入文件
- **输入**：`const std::string& filename, const std::vector<double>& data, bool scientific, int precision`
- **输出**：`bool` - 成功返回true
- **用法**：`FileIO::WriteVector("out.txt", data);`

### 7.6 CountLines
- **功能**：统计文件行数
- **输入**：`const std::string& filename`
- **输出**：`int` - 行数
- **用法**：`int lines = FileIO::CountLines("data.txt");`

### 7.7 CountColumns
- **功能**：统计文件列数（第一行）
- **输入**：`const std::string& filename`
- **输出**：`int` - 列数
- **用法**：`int cols = FileIO::CountColumns("data.txt");`

### 7.8 ToString (int)
- **功能**：整数转字符串
- **输入**：`int value`
- **输出**：`std::string` - 字符串
- **用法**：`string s = FileIO::ToString(42);`

### 7.9 ToString (double)
- **功能**：浮点数转字符串
- **输入**：`double value, int precision` - 数值、精度
- **输出**：`std::string` - 字符串
- **用法**：`string s = FileIO::ToString(3.14159, 4);`

### 7.10 Exists
- **功能**：判断文件是否存在
- **输入**：`const std::string& filename`
- **输出**：`bool` - 存在返回true
- **用法**：`bool exist = FileIO::Exists("data.txt");`

---

## 8. 数值计算模块 (Numerical)

### 8.1 RungeKutta4
- **功能**：四阶龙格-库塔数值积分
- **输入**：`const OdeFunction& f, double t0, const VectorXd& y0, double h, int steps` - 微分方程、初始时间、初始状态、步长、步数
- **输出**：`VectorXd` - 积分结果
- **用法**：`VectorXd y = Numerical::RungeKutta4(f, t0, y0, h, steps);`

### 8.2 RungeKutta45
- **功能**：自适应步长龙格-库塔积分
- **输入**：`const OdeFunction& f, double t0, const VectorXd& y0, double t_end, double tolerance`
- **输出**：`VectorXd` - 积分结果
- **用法**：`VectorXd y = Numerical::RungeKutta45(f, t0, y0, t_end, 1e-6);`

### 8.3 Bisection
- **功能**：二分法求根
- **输入**：`const std::function<double(double)>& f, double a, double b, double tol`
- **输出**：`double` - 根
- **用法**：`double root = Numerical::Bisection(f, 0.0, 5.0);`

### 8.4 Newton
- **功能**：牛顿法求根
- **输入**：`const std::function<double(double)>& f, const std::function<double(double)>& df, double x0, double tol, int max_iter`
- **输出**：`double` - 根
- **用法**：`double root = Numerical::Newton(f, df, 3.0);`

### 8.5 Derivative
- **功能**：中心差分求导数
- **输入**：`const std::function<double(double)>& f, double x, double h` - 函数、点、步长
- **输出**：`double` - 导数
- **用法**：`double deriv = Numerical::Derivative(f, 2.0);`

---

## 9. 几何计算模块 (Geometry)

### 9.1 IsPointInTriangle
- **功能**：判断点是否在三角形内
- **输入**：`const Point2D& point, const Point2D& a, const Point2D& b, const Point2D& c`
- **输出**：`bool` - 在内返回true
- **用法**：`bool inside = Geometry::IsPointInTriangle(p, a, b, c);`

### 9.2 IsPointInPolygon
- **功能**：判断点是否在多边形内
- **输入**：`const Point2D& point, const std::vector<Point2D>& polygon`
- **输出**：`bool` - 在内返回true
- **用法**：`bool inside = Geometry::IsPointInPolygon(p, polygon);`

### 9.3 DistanceToLineSegment
- **功能**：计算点到线段的距离
- **输入**：`const Point2D& point, const Point2D& line_start, const Point2D& line_end`
- **输出**：`double` - 距离
- **用法**：`double dist = Geometry::DistanceToLineSegment(p, p1, p2);`

### 9.4 Distance (2D)
- **功能**：计算两点间距离
- **输入**：`const Point2D& p1, const Point2D& p2`
- **输出**：`double` - 距离
- **用法**：`double dist = Geometry::Distance(p1, p2);`

### 9.5 PlaneNormal
- **功能**：计算三点确定的平面法向量
- **输入**：`const Vector3d& p1, const Vector3d& p2, const Vector3d& p3`
- **输出**：`Vector3d` - 法向量
- **用法**：`Vector3d normal = Geometry::PlaneNormal(p1, p2, p3);`

### 9.6 DistanceToPlane
- **功能**：计算点到平面的距离
- **输入**：`const Vector3d& point, const Vector3d& plane_point, const Vector3d& plane_normal`
- **输出**：`double` - 距离
- **用法**：`double dist = Geometry::DistanceToPlane(p, pp, pn);`

---

## 10. 时间工具模块 (Time)

### 10.1 GetUnixTimestamp
- **功能**：获取当前Unix时间戳
- **输入**：无
- **输出**：`double` - Unix时间戳（秒）
- **用法**：`double ts = Time::GetUnixTimestamp();`

### 10.2 UnixToGpsTime
- **功能**：Unix时间转GPS时间
- **输入**：`double unix_time`
- **输出**：`GpsTime` - GPS周和周内秒
- **用法**：`GpsTime gps = Time::UnixToGpsTime(unix);`

### 10.3 GpsTimeToUnix
- **功能**：GPS时间转Unix时间
- **输入**：`const GpsTime& gps_time`
- **输出**：`double` - Unix时间戳
- **用法**：`double unix = Time::GpsTimeToUnix(gps);`

### 10.4 UtcToGpsSeconds
- **功能**：UTC时间转GPS秒
- **输入**：`int year, int month, int day, int hour, int minute, double second`
- **输出**：`double` - GPS秒
- **用法**：`double gps = Time::UtcToGpsSeconds(2024, 1, 1, 12, 0, 0);`

### 10.5 GreenwhichSiderealTime
- **功能**：计算格林威治恒星时
- **输入**：`double mjd` - 修正儒略日
- **输出**：`double` - 恒星时（弧度）
- **用法**：`double gst = Time::GreenwhichSiderealTime(mjd);`

---

## 11. 单位转换模块 (Units)

### 常量定义

#### 距离
- `M_TO_FT` = 3.2808399 - 米到英尺
- `FT_TO_M` = 0.3048 - 英尺到米
- `KM_TO_M` = 1000.0 - 千米到米
- `NM_TO_M` = 1852.0 - 海里到米

#### 速度
- `KNOTS_TO_MS` = 0.514444444 - 节到米/秒
- `MS_TO_KNOTS` = 1.94384449 - 米/秒到节
- `KMH_TO_MS` = 0.277777778 - 千米/时到米/秒
- `MS_TO_KMH` = 3.6 - 米/秒到千米/时

#### 角度
- `DEG_TO_RAD` = π/180 - 度到弧度
- `RAD_TO_DEG` = 180/π - 弧度到度
- `MRAD_TO_RAD` = 0.001 - 毫弧度到弧度
- `RAD_TO_MRAD` = 1000.0 - 弧度到毫弧度

#### 质量
- `KG_TO_LB` = 2.20462262 - 千克到磅
- `LB_TO_KG` = 0.45359237 - 磅到千克

#### 力/推力
- `N_TO_LBF` = 0.224808943 - 牛顿到磅力
- `LBF_TO_N` = 4.44822162 - 磅力到牛顿
- `KN_TO_N` = 1000.0 - 千牛到牛顿

#### 压力
- `PA_TO_PSI` = 0.000145037738 - 帕斯卡到PSI
- `PSI_TO_PA` = 6894.75729 - PSI到帕斯卡
- `PA_TO_BAR` = 1e-5 - 帕斯卡到巴
- `BAR_TO_PA` = 100000.0 - 巴到帕斯卡
- `ATM_TO_PA` = 101325.0 - 标准大气压到帕斯卡

### 函数

#### CelsiusToKelvin
- **功能**：摄氏度转开尔文
- **输入**：`double c` - 摄氏度
- **输出**：`double` - 开尔文
- **用法**：`double k = Units::CelsiusToKelvin(25.0);`

#### KelvinToCelsius
- **功能**：开尔文转摄氏度
- **输入**：`double k` - 开尔文
- **输出**：`double` - 摄氏度
- **用法**：`double c = Units::KelvinToCelsius(298.15);`

#### FahrenheitToKelvin
- **功能**：华氏度转开尔文
- **输入**：`double f` - 华氏度
- **输出**：`double` - 开尔文
- **用法**：`double k = Units::FahrenheitToKelvin(77.0);`

---

## 12. 矩阵运算模块 (MatrixUtils)

### 12.1 Multiply (向量)
- **功能**：3x3矩阵乘以3维向量
- **输入**：`const double A[3][3], const double B[3], double C[3]`
- **输出**：通过引用返回结果
- **用法**：`MatrixUtils::Multiply(A, b, c);`

### 12.2 Multiply3x3
- **功能**：两个3x3矩阵相乘
- **输入**：`const double A[3][3], const double B[3][3], double C[3][3]`
- **输出**：通过引用返回结果
- **用法**：`MatrixUtils::Multiply3x3(A, B, C);`

### 12.3 Transpose
- **功能**：3x3矩阵转置
- **输入**：`const double A[3][3], double B[3][3]`
- **输出**：通过引用返回转置矩阵
- **用法**：`MatrixUtils::Transpose(A, B);`

### 12.4 OuterProduct
- **功能**：计算两个向量的外积
- **输入**：`const Vector3d& a, const Vector3d& b`
- **输出**：`Matrix3d` - 外积矩阵
- **用法**：`Matrix3d M = MatrixUtils::OuterProduct(a, b);`

### 12.5 SkewSymmetric
- **功能**：创建斜对称矩阵（用于叉乘）
- **输入**：`const Vector3d& v`
- **输出**：`Matrix3d` - 斜对称矩阵
- **用法**：`Matrix3d S = MatrixUtils::SkewSymmetric(v);`

### 12.6 QuaternionToMatrix
- **功能**：四元数转旋转矩阵
- **输入**：`const Vector4d& q` - 四元数 (w, x, y, z)
- **输出**：`Matrix3d` - 旋转矩阵
- **用法**：`Matrix3d R = MatrixUtils::QuaternionToMatrix(q);`

### 12.7 MatrixToEuler
- **功能**：旋转矩阵转欧拉角
- **输入**：`const Matrix3d& R` - 旋转矩阵
- **输出**：`Vector3d` - (roll, pitch, yaw)
- **用法**：`Vector3d euler = MatrixUtils::MatrixToEuler(R);`

### 12.8 EulerToMatrix
- **功能**：欧拉角转旋转矩阵
- **输入**：`double roll, double pitch, double yaw`
- **输出**：`Matrix3d` - 旋转矩阵
- **用法**：`Matrix3d R = MatrixUtils::EulerToMatrix(0.1, 0.2, 0.3);`

---

## 13. 仿真实用工具模块 (Simulation)

### 13.1 Timer（计时器类）

#### 构造/析构
- **Timer()**: 创建计时器
- **~Timer()**: 销毁计时器

#### Start
- **功能**：开始计时
- **输入**：无
- **输出**：无
- **用法**：`timer.Start();`

#### Stop
- **功能**：停止计时
- **输入**：无
- **输出**：无
- **用法**：`timer.Stop();`

#### ElapsedSeconds
- **功能**：获取经过的秒数
- **输入**：无
- **输出**：`double` - 秒数
- **用法**：`double sec = timer.ElapsedSeconds();`

#### ElapsedMilliseconds
- **功能**：获取经过的毫秒数
- **输入**：无
- **输出**：`double` - 毫秒数
- **用法**：`double ms = timer.ElapsedMilliseconds();`

### 13.2 Logger（日志类）

#### Log
- **功能**：记录日志
- **输入**：`LogLevel level, const std::string& message`
- **输出**：无
- **用法**：`Simulation::Logger::Log(Simulation::LogLevel::Info, "Message");`

#### Debug
- **功能**：记录调试日志
- **输入**：`const std::string& message`
- **输出**：无
- **用法**：`Simulation::Logger::Debug("Debug info");`

#### Info
- **功能**：记录信息日志
- **输入**：`const std::string& message`
- **输出**：无
- **用法**：`Simulation::Logger::Info("Info message");`

#### Warning
- **功能**：记录警告日志
- **输入**：`const std::string& message`
- **输出**：无
- **用法**：`Simulation::Logger::Warning("Warning message");`

#### Error
- **功能**：记录错误日志
- **输入**：`const std::string& message`
- **输出**：无
- **用法**：`Simulation::Logger::Error("Error message");`

#### SetLogLevel
- **功能**：设置日志级别
- **输入**：`LogLevel level`
- **输出**：无
- **用法**：`Simulation::Logger::SetLogLevel(Simulation::LogLevel::Info);`

#### SetOutputFile
- **功能**：设置日志输出文件
- **输入**：`const std::string& filename`
- **输出**：无
- **用法**：`Simulation::Logger::SetOutputFile("log.txt");`

---

## 14. 版本信息模块 (Version)

### 常量

- **MAJOR** = 2 - 主版本号
- **MINOR** = 0 - 次版本号
- **PATCH** = 0 - 补丁号

### ToString
- **功能**：获取版本字符串
- **输入**：无
- **输出**：`std::string` - 版本号字符串
- **用法**：`string ver = Version::ToString();` // 返回 "2.0.0"

---

## 统计总结

**SimTools v2.0 共实现了 120+ 个功能函数，分布在 14 个模块中：**

| 模块 | 函数数量 | 主要功能 |
|------|---------|---------|
| Math | 11 | 基础数学运算、向量运算、角度规范化 |
| Interpolation | 7 | 一维/二维插值算法 |
| Coordinate | 14 | GPS/ECEF/NED坐标转换、速度转换 |
| Geodesy | 10 | 地理距离、方位角、Vincenty计算 |
| Atmosphere | 9 | 大气参数计算、马赫数转换 |
| Random | 8 | 各种分布的随机数生成 |
| FileIO | 10 | 文件读写、数据统计 |
| Numerical | 5 | 数值积分、求根、微分 |
| Geometry | 6 | 几何判断、距离计算 |
| Time | 5 | 时间转换、GPS时间 |
| Units | 3+16 | 单位转换常量及函数 |
| MatrixUtils | 8 | 矩阵运算、欧拉角/四元数转换 |
| Simulation | 12 | 计时器、日志系统 |
| Version | 4 | 版本信息 |

### 完整函数统计表

#### 1. Math 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| Sign | 返回数值的符号 | T x | T (-1/0/1) |
| Max | 返回两个数的最大值 | T a, T b | T |
| Min | 返回两个数的最小值 | T a, T b | T |
| Clamp | 将值限制在范围内 | T value, T min_val, T max_val | T |
| FindAbsMax | 查找绝对值最大的元素 | vector<T>& data, int& index | T |
| Norm2 (Vector3d) | 计算3D向量长度 | Vector3d vec | double |
| Norm2 (VectorXd) | 计算动态向量长度 | VectorXd vec | double |
| Normalize | 向量归一化 | Vector3d vec | Vector3d |
| Regulate180 | 角度规范化到[-180°,180°] | double angle | double |
| Regulate360 | 角度规范化到[0°,360°] | double angle | double |
| RegulatePi | 弧度规范化到[-π,π] | double angle | double |

#### 2. Interpolation 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| Linear | 一维线性插值 | double x, vector<double>& xx, vector<double>& yy | double |
| Lagrange7 | 7点拉格朗日插值 | double x, vector<double>& xx, vector<double>& yy | double |
| LagrangeGlobal | 全局拉格朗日插值 | double x, vector<double>& xx, vector<double>& yy | double |
| CubicSpline | 三次样条插值 | double x, vector<double>& xx, vector<double>& yy | double |
| Bilinear | 双线性二维插值 | double u, v, vector<double>& x, y, vector<vector<double>>& z | double |
| FindIndex | 查找插值索引 | double x, vector<double>& xx | int |
| FindIndex2D | 查找二维插值索引 | double u, v, vector<double>& x, y | pair<int,int> |

#### 3. Coordinate 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| GpsToEcef | GPS转ECEF坐标 | Vector3(经°,纬°,高m) | Vector3(X,Y,Z)m |
| EcefToGps | ECEF转GPS坐标 | Vector3(X,Y,Z)m | Vector3(经°,纬°,高m) |
| EcefToGpsNewton | ECEF转GPS(牛顿法) | Vector3(X,Y,Z)m | Vector3(经°,纬°,高m) |
| EcefToNedMatrix | ECEF到NED旋转矩阵 | double lon, lat | Matrix3 |
| NedToEcefMatrix | NED到ECEF旋转矩阵 | double lon, lat | Matrix3 |
| EcefToNed | ECEF转NED位置 | Vector3 ecef_pos, Vector3 ref_gps | Vector3(北,东,地)m |
| NedToEcef | NED转ECEF位置 | Vector3 ned_pos, Vector3 ref_gps | Vector3(X,Y,Z)m |
| VelocityToNed | 弹道速度转NED速度 | double theta, phi_v, V, Vector3& vn | void(引用返回) |
| NedToEcefVelocity | NED速度转ECEF速度 | Vector3 ned_vel, gps_pos, Vector3& ecef_vel | void(引用返回) |
| RotationMatrix | 基本旋转矩阵 | double angle, int axis(1/2/3) | Matrix3 |
| DmsToDecimal | 度分秒转十进制 | double degree, minute, second | double |
| DecimalToDms | 十进制转度分秒 | double decimal, int& deg, min, double& sec | void(引用返回) |
| RadToDeg | 弧度转角度 | double rad | double |
| DegToRad | 角度转弧度 | double deg | double |

#### 4. Geodesy 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| GreatCircleDistance | 大圆距离(球面) | double lon1,lat1,lon2,lat2 | double(米) |
| HaversineDistance | Haversine距离 | double lon1,lat1,lon2,lat2 | double(米) |
| VincentyDistance | Vincenty距离(椭球) | double lon1,lat1,lon2,lat2 | double(米) |
| Azimuth | 初始方位角 | Vector3d gps1, gps2 | double(度) |
| AzimuthAndDistance | 方位角和距离 | Vector3d gps1, gps2, double& az, dist | void(引用返回) |
| VincentyInverse | Vincenty反解 | double lon1,lat1,lon2,lat2, double& dist, az1, az2 | void(引用返回) |
| VincentyDirect | Vincenty正解 | double lon1,lat1,az,dist, double& lon2,lat2,az2 | void(引用返回) |
| TargetFromSite | 站心求目标GPS | Vector3d site_gps, double az, h, range | Vector3d |
| SiteAzimuth | 站心方位角 | Vector3d gpsA, gpsB | double(度) |
| SiteDistance | 站心斜距 | Vector3d gpsA, gpsB | double(米) |

#### 5. Atmosphere 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| GetParameters | 完整大气参数 | double height_meters | Parameters结构体 |
| Gravity | 重力加速度 | double height_meters | double(m/s²) |
| SoundSpeed | 声速 | double height_meters | double(m/s) |
| Density | 空气密度 | double height_meters | double(kg/m³) |
| Pressure | 气压 | double height_meters | double(Pa) |
| Temperature | 温度 | double height_meters | double(K) |
| VelocityFromMach | 马赫数转速度 | double mach, height_meters | double(m/s) |
| MachFromVelocity | 速度转马赫数 | double velocity, height_meters | double |
| DynamicPressure | 动压 | double velocity, height_meters | double(Pa) |

#### 6. Random 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| Seed | 设置随机种子 | unsigned int seed(0=当前时间) | void |
| Uniform01 | [0,1)均匀分布 | 无 | double |
| Uniform | [a,b]均匀分布 | double a, b | double |
| UniformInt | 整数均匀分布 | int a, b | int |
| Normal01 | 标准正态分布N(0,1) | 无 | double |
| Normal | 正态分布N(μ,σ²) | double mu, sigma | double |
| Exponential | 指数分布 | double lambda | double |
| Weibull | 韦伯分布 | double shape, scale | double |

#### 7. FileIO 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| ReadMatrix | 读取到矩阵 | string filename | MatrixXd |
| ReadTable | 读取到二维表 | string filename | vector<vector<double>> |
| ReadColumn | 读取单列 | string filename, int column_index | vector<double> |
| WriteMatrix | 写入矩阵 | string filename, MatrixXd, bool scientific, int precision | bool |
| WriteVector | 写入向量 | string filename, vector<double>, bool scientific, int precision | bool |
| CountLines | 统计行数 | string filename | int |
| CountColumns | 统计列数 | string filename | int |
| ToString(int) | 整数转字符串 | int value | string |
| ToString(double) | 浮点数转字符串 | double value, int precision | string |
| Exists | 文件是否存在 | string filename | bool |

#### 8. Numerical 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| RungeKutta4 | 四阶龙格-库塔积分 | OdeFunction f, double t0, VectorXd y0, double h, int steps | VectorXd |
| RungeKutta45 | 自适应RK积分 | OdeFunction f, double t0, VectorXd y0, double t_end, double tol | VectorXd |
| Bisection | 二分法求根 | function<double(double)> f, double a, b, double tol | double |
| Newton | 牛顿法求根 | function<double(double)> f, df, double x0, double tol, int max_iter | double |
| Derivative | 中心差分求导 | function<double(double)> f, double x, double h | double |

#### 9. Geometry 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| IsPointInTriangle | 点在三角形内判断 | Point2D point, a, b, c | bool |
| IsPointInPolygon | 点在多边形内判断 | Point2D point, vector<Point2D> polygon | bool |
| DistanceToLineSegment | 点到线段距离 | Point2D point, line_start, line_end | double |
| Distance(2D) | 两点距离 | Point2D p1, p2 | double |
| PlaneNormal | 平面法向量 | Vector3d p1, p2, p3 | Vector3d |
| DistanceToPlane | 点到平面距离 | Vector3d point, plane_point, plane_normal | double |

#### 10. Time 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| GetUnixTimestamp | Unix时间戳 | 无 | double(秒) |
| UnixToGpsTime | Unix转GPS时间 | double unix_time | GpsTime(周,周内秒) |
| GpsTimeToUnix | GPS时间转Unix | GpsTime gps_time | double(秒) |
| UtcToGpsSeconds | UTC转GPS秒 | int year,month,day,hour,min, double second | double |
| GreenwhichSiderealTime | 格林威治恒星时 | double mjd(修正儒略日) | double(弧度) |

#### 11. Units 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| CelsiusToKelvin | 摄氏度转开尔文 | double c | double K |
| KelvinToCelsius | 开尔文转摄氏度 | double k | double °C |
| FahrenheitToKelvin | 华氏度转开尔文 | double f | double K |

**常量定义**（16个）：
- 距离: `M_TO_FT`, `FT_TO_M`, `KM_TO_M`, `NM_TO_M`
- 速度: `KNOTS_TO_MS`, `MS_TO_KNOTS`, `KMH_TO_MS`, `MS_TO_KMH`
- 角度: `DEG_TO_RAD`, `RAD_TO_DEG`, `MRAD_TO_RAD`, `RAD_TO_MRAD`
- 质量: `KG_TO_LB`, `LB_TO_KG`
- 力: `N_TO_LBF`, `LBF_TO_N`, `KN_TO_N`
- 压力: `PA_TO_PSI`, `PSI_TO_PA`, `PA_TO_BAR`, `BAR_TO_PA`, `ATM_TO_PA`

#### 12. MatrixUtils 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| Multiply | 矩阵乘向量 | double A[3][3], B[3], C[3] | void(引用返回) |
| Multiply3x3 | 矩阵乘矩阵 | double A[3][3], B[3][3], C[3][3] | void(引用返回) |
| Transpose | 矩阵转置 | double A[3][3], B[3][3] | void(引用返回) |
| OuterProduct | 向量外积 | Vector3d a, b | Matrix3d |
| SkewSymmetric | 斜对称矩阵 | Vector3d v | Matrix3d |
| QuaternionToMatrix | 四元数转旋转矩阵 | Vector4d q(w,x,y,z) | Matrix3d |
| MatrixToEuler | 矩阵转欧拉角 | Matrix3d R | Vector3d(roll,pitch,yaw) |
| EulerToMatrix | 欧拉角转矩阵 | double roll, pitch, yaw | Matrix3d |

#### 13. Simulation 模块

**Timer 类（6个方法）**：

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| Timer() | 构造计时器 | 无 | Timer对象 |
| ~Timer() | 析构计时器 | 无 | void |
| Start | 开始计时 | 无 | void |
| Stop | 停止计时 | 无 | void |
| ElapsedSeconds | 经过秒数 | 无 | double |
| ElapsedMilliseconds | 经过毫秒数 | 无 | double |

**Logger 类（6个方法）**：

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| Log | 记录日志 | LogLevel level, string message | void |
| Debug | 调试日志 | string message | void |
| Info | 信息日志 | string message | void |
| Warning | 警告日志 | string message | void |
| Error | 错误日志 | string message | void |
| SetLogLevel | 设置日志级别 | LogLevel level | void |
| SetOutputFile | 设置输出文件 | string filename | void |

#### 14. Version 模块

| 函数名 | 函数功能 | 输入 | 输出 |
|--------|---------|------|------|
| MAJOR | 主版本号常量 | 无 | int(2) |
| MINOR | 次版本号常量 | 无 | int(0) |
| PATCH | 补丁号常量 | 无 | int(0) |
| ToString | 版本字符串 | 无 | string("2.0.0") |

---

## 使用示例

### 示例1：坐标转换与地理计算
```cpp
#include "SimTools_v2.h"
using namespace SimTools;

// GPS坐标
Vector3d beijing(116.3974, 39.9093, 100.0);
Vector3d shanghai(121.4737, 31.2304, 0.0);

// 计算距离和方位角
double dist = Geodesy::GreatCircleDistance(
    beijing[0], beijing[1],
    shanghai[0], shanghai[1]
);
double az = Geodesy::Azimuth(beijing, shanghai);

// 转换到NED坐标
Vector3d ecef = Coordinate::GpsToEcef(shanghai);
Vector3d ned = Coordinate::EcefToNed(ecef, beijing);
```

### 示例2：大气参数计算
```cpp
// 获取10km高度的大气参数
auto air = Atmosphere::GetParameters(10000.0);
std::cout << "温度: " << air.temperature << " K" << std::endl;
std::cout << "气压: " << air.pressure << " Pa" << std::endl;
std::cout << "密度: " << air.density << " kg/m³" << std::endl;

// 马赫数2.0对应的速度
double velocity = Atmosphere::VelocityFromMach(2.0, 10000.0);
```

### 示例3：数值积分
```cpp
// 定义微分方程: dy/dt = -y
auto f = [](double t, const VectorXd& y) -> VectorXd {
    return VectorXd{-y[0]};
};

VectorXd y0(1);
y0[0] = 1.0;

// 使用RK4积分
VectorXd result = Numerical::RungeKutta4(f, 0.0, y0, 0.1, 10);
```

### 示例4：随机数与统计
```cpp
Random::Seed(42);

// 生成正态分布随机数
for (int i = 0; i < 100; i++) {
    double r = Random::Normal(100.0, 15.0);
    // 处理随机数...
}
```

### 示例5：文件操作
```cpp
// 写入数据
std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};
FileIO::WriteVector("output.txt", data);

// 读取数据
auto loaded = FileIO::ReadColumn("output.txt", 0);

// 统计信息
int lines = FileIO::CountLines("output.txt");
int cols = FileIO::CountColumns("output.txt");
```

---

## 注意事项

1. **头文件包含**：所有功能通过 `#include "SimTools_v2.h"` 即可使用
2. **命名空间**：所有功能都在 `SimTools` 命名空间下
3. **类型定义**：使用 Eigen 库时，Vector3d、Matrix3d 等类型自动使用 Eigen 类型
4. **线程安全**：Random 模块的 Seed 函数影响全局状态，多线程使用时需注意
5. **精度控制**：数值计算函数都有默认容差，可根据需要调整

---

**文档版本**: v2.0.0
**最后更新**: 2026年2月6日
