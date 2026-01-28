#pragma once

#include <vector>
#include <string>
#include <array>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <memory>
#include <functional>
#include <utility>

// ============================================================
// Eigen 库检测与兼容层
// ============================================================

#ifdef USE_EIGEN
    #include <Eigen/Dense>
    namespace SimTools {
        using Vector3d = Eigen::Vector3d;
        using Matrix3d = Eigen::Matrix3d;
        using VectorXd = Eigen::VectorXd;
        using MatrixXd = Eigen::MatrixXd;
        using Vector4d = Eigen::Vector4d;
    }
#else
    // 如果没有 Eigen，使用标准库替代
    #include <valarray>
    namespace SimTools {
        // 3D 向量类型
        struct Vector3d {
            double data[3];

            Vector3d() : data{0, 0, 0} {}
            Vector3d(double x, double y, double z) : data{x, y, z} {}
            Vector3d(double val) : data{val, val, val} {}

            // 静态方法：创建零向量
            static Vector3d Zero() {
                return Vector3d(0, 0, 0);
            }

            inline double& operator[](int i) { return data[i]; }
            inline const double& operator[](int i) const { return data[i]; }

            inline Vector3d operator-(const Vector3d& other) const {
                return Vector3d(data[0] - other.data[0],
                               data[1] - other.data[1],
                               data[2] - other.data[2]);
            }

            inline Vector3d operator+(const Vector3d& other) const {
                return Vector3d(data[0] + other.data[0],
                               data[1] + other.data[1],
                               data[2] + other.data[2]);
            }

            inline Vector3d operator*(double scalar) const {
                return Vector3d(data[0] * scalar, data[1] * scalar, data[2] * scalar);
            }

            inline Vector3d& operator*=(double scalar) {
                data[0] *= scalar;
                data[1] *= scalar;
                data[2] *= scalar;
                return *this;
            }

            inline Vector3d operator/(double scalar) const {
                return Vector3d(data[0] / scalar, data[1] / scalar, data[2] / scalar);
            }

            inline Vector3d& operator/=(double scalar) {
                data[0] /= scalar;
                data[1] /= scalar;
                data[2] /= scalar;
                return *this;
            }

            inline double dot(const Vector3d& other) const {
                return data[0] * other.data[0] + data[1] * other.data[1] + data[2] * other.data[2];
            }

            inline Vector3d cross(const Vector3d& other) const {
                return Vector3d(
                    data[1] * other.data[2] - data[2] * other.data[1],
                    data[2] * other.data[0] - data[0] * other.data[2],
                    data[0] * other.data[1] - data[1] * other.data[0]
                );
            }

            inline double norm() const {
                return std::sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
            }

            inline Vector3d normalized() const {
                double n = norm();
                if (n > 1e-10) {
                    return Vector3d(data[0] / n, data[1] / n, data[2] / n);
                }
                return Vector3d(0, 0, 0);
            }

            inline int size() const { return 3; }

            inline double x() const { return data[0]; }
            inline double y() const { return data[1]; }
            inline double z() const { return data[2]; }

            // 添加 transpose() 以兼容 Eigen 接口
            inline Vector3d transpose() const {
                return *this;
            }

            // 添加 cwiseAbs() 以兼容 Eigen 接口
            inline Vector3d cwiseAbs() const {
                return Vector3d(std::abs(data[0]), std::abs(data[1]), std::abs(data[2]));
            }
        };

        // 3x3 矩阵类型
        struct Matrix3d {
            double data[9];  // 行主序存储

            Matrix3d() {
                for (int i = 0; i < 9; i++) data[i] = 0;
                // 单位矩阵
                data[0] = data[4] = data[8] = 1.0;
            }

            // 静态方法：创建单位矩阵
            static Matrix3d Identity() {
                return Matrix3d();
            }

            inline double& operator()(int row, int col) {
                return data[row * 3 + col];
            }

            inline const double& operator()(int row, int col) const {
                return data[row * 3 + col];
            }

            inline Matrix3d operator*(const Matrix3d& other) const {
                Matrix3d result;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        result(i, j) = 0;
                        for (int k = 0; k < 3; k++) {
                            result(i, j) += (*this)(i, k) * other(k, j);
                        }
                    }
                }
                return result;
            }

            inline Vector3d operator*(const Vector3d& vec) const {
                return Vector3d(
                    data[0] * vec[0] + data[1] * vec[1] + data[2] * vec[2],
                    data[3] * vec[0] + data[4] * vec[1] + data[5] * vec[2],
                    data[6] * vec[0] + data[7] * vec[1] + data[8] * vec[2]
                );
            }

            inline Matrix3d transpose() const {
                Matrix3d result;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        result(i, j) = (*this)(j, i);
                    }
                }
                return result;
            }

            inline Matrix3d operator+(const Matrix3d& other) const {
                Matrix3d result;
                for (int i = 0; i < 9; i++) {
                    result.data[i] = data[i] + other.data[i];
                }
                return result;
            }

            inline double trace() const {
                return data[0] + data[4] + data[8];
            }

            inline double norm() const {
                double sum = 0;
                for (int i = 0; i < 9; i++) {
                    sum += data[i] * data[i];
                }
                return std::sqrt(sum);
            }
        };

        // 动态大小向量（简化版）
        struct VectorXd : public std::vector<double> {
            using std::vector<double>::vector;

            inline VectorXd operator-(const VectorXd& other) const {
                VectorXd result(this->size());
                for (size_t i = 0; i < static_cast<size_t>(this->size()); i++) {
                    result[i] = (*this)[i] - other[i];
                }
                return result;
            }

            inline VectorXd operator+(const VectorXd& other) const {
                VectorXd result(this->size());
                for (size_t i = 0; i < static_cast<size_t>(this->size()); i++) {
                    result[i] = (*this)[i] + other[i];
                }
                return result;
            }

            inline VectorXd operator*(double scalar) const {
                VectorXd result(this->size());
                for (size_t i = 0; i < static_cast<size_t>(this->size()); i++) {
                    result[i] = (*this)[i] * scalar;
                }
                return result;
            }

            inline VectorXd operator/(double scalar) const {
                VectorXd result(this->size());
                for (size_t i = 0; i < static_cast<size_t>(this->size()); i++) {
                    result[i] = (*this)[i] / scalar;
                }
                return result;
            }

            inline VectorXd& operator*=(double scalar) {
                for (size_t i = 0; i < static_cast<size_t>(this->size()); i++) {
                    (*this)[i] *= scalar;
                }
                return *this;
            }

            inline VectorXd& operator+=(const VectorXd& other) {
                for (size_t i = 0; i < static_cast<size_t>(this->size()); i++) {
                    (*this)[i] += other[i];
                }
                return *this;
            }

            inline double norm() const {
                double sum = 0;
                for (const auto& val : *this) {
                    sum += val * val;
                }
                return std::sqrt(sum);
            }

            inline int size() const {
                return static_cast<int>(std::vector<double>::size());
            }

            // 返回每个元素的绝对值
            inline VectorXd cwiseAbs() const {
                VectorXd result(this->size());
                for (size_t i = 0; i < static_cast<size_t>(this->size()); i++) {
                    result[i] = std::abs((*this)[i]);
                }
                return result;
            }

            // 返回最大值系数
            inline double maxCoeff() const {
                if (this->empty()) return 0.0;
                double maxVal = (*this)[0];
                for (size_t i = 1; i < static_cast<size_t>(this->size()); i++) {
                    if ((*this)[i] > maxVal) {
                        maxVal = (*this)[i];
                    }
                }
                return maxVal;
            }
        };

        // 全局运算符：double * VectorXd
        inline VectorXd operator*(double scalar, const VectorXd& vec) {
            return vec * scalar;
        }

        // 4D 向量（四元数）
        struct Vector4d {
            double data[4];

            Vector4d() : data{0, 0, 0, 0} {}
            Vector4d(double w, double x, double y, double z) : data{w, x, y, z} {}

            inline double& operator[](int i) { return data[i]; }
            inline const double& operator[](int i) const { return data[i]; }

            inline Vector4d normalized() const {
                double n = std::sqrt(data[0]*data[0] + data[1]*data[1] +
                                    data[2]*data[2] + data[3]*data[3]);
                if (n > 1e-10) {
                    return Vector4d(data[0]/n, data[1]/n, data[2]/n, data[3]/n);
                }
                return Vector4d(0, 0, 0, 0);
            }
        };

        // 动态矩阵（简化版，仅用于兼容）
        struct MatrixXd {
            std::vector<std::vector<double>> data;

            MatrixXd(int rows, int cols) : data(rows, std::vector<double>(cols)) {}

            inline int rows() const { return static_cast<int>(data.size()); }
            inline int cols() const { return data.empty() ? 0 : static_cast<int>(data[0].size()); }

            inline double& operator()(int row, int col) { return data[row][col]; }
            inline const double& operator()(int row, int col) const { return data[row][col]; }
        };
    }
#endif

// ============================================================
// 平台导出宏定义
// ============================================================

#ifdef SIMTOOLS_STATIC
    #define SIMTOOLS_API
#else
    #ifdef _WIN32
        #ifdef SIMTOOLS_EXPORTS
            #define SIMTOOLS_API __declspec(dllexport)
        #else
            #define SIMTOOLS_API __declspec(dllimport)
        #endif
    #else
        #define SIMTOOLS_API __attribute__((visibility("default")))
    #endif
#endif

// ============================================================
// 常量定义
// ============================================================

namespace SimTools {
    namespace Constants {
        // 数学常量
        constexpr double PI = 3.14159265358979323846;
        constexpr double RAD_TO_DEG = 180.0 / PI;
        constexpr double DEG_TO_RAD = PI / 180.0;

        // CGCS2000 坐标系参数
        constexpr double EARTH_SEMIMAJOR = 6378137.0;          // 长半轴 (m)
        constexpr double EARTH_SEMIMINOR = 6356752.3141;       // 短半轴 (m)
        constexpr double EARTH_FLATTENING = 1.0 / 298.257222101;
        constexpr double ECCENTRICITY_FIRST = 0.081819191042816;
        constexpr double ECCENTRICITY_SECOND = 0.082094438151917;

        // 计算精度
        constexpr double EPSILON = 1e-10;
        constexpr double ITERATION_TOL = 1e-10;

        // 物理常量
        constexpr double GRAVITY_SEA_LEVEL = 9.80665;         // m/s²
        constexpr double GAS_CONSTANT_AIR = 287.05287;        // J/(kg·K)
        constexpr double STANDARD_PRESSURE = 101325.0;        // Pa
        constexpr double STANDARD_TEMPERATURE = 288.15;       // K
        constexpr double STANDARD_DENSITY = 1.225;            // kg/m³
    }
}

// ============================================================
// 数学工具模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Math {
        // ===== 基础数学函数 =====

        // 符号函数：返回 x 的符号 (-1, 0, 1)
        template<typename T>
        static inline T Sign(T x) {
            return (T(0) < x) - (x < T(0));
        }

        // 最大值
        template<typename T>
        static inline T Max(T a, T b) {
            return (a >= b) ? a : b;
        }

        // 最小值
        template<typename T>
        static inline T Min(T a, T b) {
            return (a <= b) ? a : b;
        }

        // 限制值在范围内
        template<typename T>
        static inline T Clamp(T value, T min_val, T max_val) {
            return Max(min_val, Min(value, max_val));
        }

        // 查找绝对值最大值及其索引
        template<typename T>
        static T FindAbsMax(const std::vector<T>& data, int& index) {
            if (data.empty()) {
                index = -1;
                return T(0);
            }

            T maxVal = data[0];
            index = 0;
            for (size_t i = 1; i < data.size(); ++i) {
                if (std::abs(data[i]) > std::abs(maxVal)) {
                    maxVal = data[i];
                    index = static_cast<int>(i);
                }
            }
            return maxVal;
        }

        // 向量二范数
        static inline double Norm2(const Vector3d& vec) {
            return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
        }

        static inline double Norm2(const VectorXd& vec) {
            return vec.norm();
        }

        // 向量归一化
        static inline Vector3d Normalize(const Vector3d& vec) {
            double n = Norm2(vec);
            if (n < Constants::EPSILON) {
                return Vector3d::Zero();
            }
            return vec / n;
        }

        // 角度规范化到 [-180, 180]
        static inline double Regulate180(double angle) {
            double temp = angle - 360.0 * std::floor(angle / 360.0);
            if (std::abs(temp) > 180.0) {
                angle = temp - 360.0 * Sign(temp);
            } else {
                angle = temp;
            }
            return angle;
        }

        // 角度规范化到 [0, 360]
        static inline double Regulate360(double angle) {
            angle = angle - 360.0 * std::floor(angle / 360.0);
            if (angle < 0) {
                angle += 360.0;
            }
            return angle;
        }

        // 弧度规范化到 [-PI, PI]
        static inline double RegulatePi(double angle) {
            double temp = angle - 2 * Constants::PI * std::floor(angle / (2 * Constants::PI));
            if (std::abs(temp) > Constants::PI) {
                angle = temp - 2 * Constants::PI * Sign(temp);
            } else {
                angle = temp;
            }
            return angle;
        }
    };
}

// ============================================================
// 插值算法模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Interpolation {
        // ===== 一维插值 =====

        // 线性插值
        static double Linear(double x,
                            const std::vector<double>& xx,
                            const std::vector<double>& yy);

        // 拉格朗日插值（使用前后7个点）
        static double Lagrange7(double x,
                                const std::vector<double>& xx,
                                const std::vector<double>& yy);

        // 全局拉格朗日插值
        static double LagrangeGlobal(double x,
                                    const std::vector<double>& xx,
                                    const std::vector<double>& yy);

        // 三次样条插值
        static double CubicSpline(double x,
                                 const std::vector<double>& xx,
                                 const std::vector<double>& yy);

        // ===== 二维插值 =====

        // 双线性插值
        static double Bilinear(double u, double v,
                              const std::vector<double>& x,
                              const std::vector<double>& y,
                              const std::vector<std::vector<double>>& z);

        // ===== 查找插值索引 =====

        // 找到 x 在 xx 中的位置（返回大于 x 的第一个索引）
        static int FindIndex(double x, const std::vector<double>& xx);

        // 找到二维插值的索引
        static std::pair<int, int> FindIndex2D(double u, double v,
                                              const std::vector<double>& x,
                                              const std::vector<double>& y);
    };
}

// ============================================================
// 坐标转换模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Coordinate {
        // 类型定义
        using Vector3 = Vector3d;
        using Matrix3 = Matrix3d;

        // ===== 基础坐标系统 =====

        // GPS (经度°, 纬度°, 高度m) -> ECEF (X, Y, Z)m
        static Vector3 GpsToEcef(const Vector3& gps);

        // ECEF (X, Y, Z)m -> GPS (经度°, 纬度°, 高度m)
        static Vector3 EcefToGps(const Vector3& ecef);

        // ECEF -> GPS (使用牛顿迭代，精度更高)
        static Vector3 EcefToGpsNewton(const Vector3& ecef);

        // ===== 局部切平面坐标系 =====

        // 创建 ECEF 到 NED (北东地) 的旋转矩阵
        static Matrix3 EcefToNedMatrix(double longitude, double latitude);

        // 创建 NED 到 ECEF 的旋转矩阵
        static Matrix3 NedToEcefMatrix(double longitude, double latitude);

        // ECEF 位置 -> NED 位置（相对于参考点）
        static Vector3 EcefToNed(const Vector3& ecef_pos,
                                const Vector3& ref_gps);

        // NED 位置 -> ECEF 位置（相对于参考点）
        static Vector3 NedToEcef(const Vector3& ned_pos,
                                const Vector3& ref_gps);

        // ===== 速度转换 =====

        // 从弹道坐标系速度到 NED 速度
        // theta: 弹道倾斜角, phi_v: 弹道偏角, V: 速度
        static void VelocityToNed(double theta, double phi_v, double V,
                                 Vector3& vn);

        // NED 速度 -> ECEF 速度
        static void NedToEcefVelocity(const Vector3& ned_vel,
                                     const Vector3& gps_pos,
                                     Vector3& ecef_vel);

        // ===== 坐标旋转矩阵 =====

        // 创建基本旋转矩阵 (绕 X/Y/Z 轴)
        // axis: 1=X, 2=Y, 3=Z
        static Matrix3 RotationMatrix(double angle, int axis);

        // ===== 角度格式转换 =====

        // 度分秒 -> 十进制
        static inline double DmsToDecimal(double degree, double minute, double second) {
            return degree + minute / 60.0 + second / 3600.0;
        }

        // 十进制 -> 度分秒
        static inline void DecimalToDms(double decimal,
                                       int& degree, int& minute, double& second) {
            degree = static_cast<int>(decimal);
            minute = static_cast<int>((decimal - degree) * 3600.0 / 60.0);
            second = (decimal - degree - minute / 60.0) * 3600.0;
        }

        // 弧度 -> 角度
        static inline double RadToDeg(double rad) {
            return rad * Constants::RAD_TO_DEG;
        }

        // 角度 -> 弧度
        static inline double DegToRad(double deg) {
            return deg * Constants::DEG_TO_RAD;
        }
    };
}

// ============================================================
// 地理计算模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Geodesy {
        // ===== 距离计算 =====

        // 大圆距离（球面模型，快速但精度较低）
        static double GreatCircleDistance(double lon1, double lat1,
                                         double lon2, double lat2);

        // Haversine 距离（球面模型）
        static double HaversineDistance(double lon1, double lat1,
                                       double lon2, double lat2);

        // Vincenty 距离（椭球面模型，高精度）
        static double VincentyDistance(double lon1, double lat1,
                                      double lon2, double lat2);

        // ===== 方位角计算 =====

        // 计算初始方位角（从点1到点2）
        static double Azimuth(const Vector3d& gps1,
                             const Vector3d& gps2);

        // 计算两点间方位角和距离
        static void AzimuthAndDistance(const Vector3d& gps1,
                                      const Vector3d& gps2,
                                      double& azimuth,
                                      double& distance);

        // ===== Vincenty 正反解 =====

        // Vincenty 反解：已知两点坐标，求距离和方位角
        static void VincentyInverse(double lon1, double lat1,
                                   double lon2, double lat2,
                                   double& distance,
                                   double& azimuth1,
                                   double& azimuth2);

        // Vincenty 正解：已知起点、方位角和距离，求终点
        static void VincentyDirect(double lon1, double lat1,
                                 double azimuth, double distance,
                                 double& lon2, double& lat2,
                                 double& azimuth2);

        // ===== 站心计算 =====

        // 根据观测站坐标、方位角、高度和斜距计算目标GPS坐标
        static Vector3d TargetFromSite(const Vector3d& site_gps,
                                             double azimuth,
                                             double target_height,
                                             double slant_range);

        // 计算两GPS点间的站心方位角
        static double SiteAzimuth(const Vector3d& gpsA,
                                 const Vector3d& gpsB);

        // 计算两GPS点间的站心斜距
        static double SiteDistance(const Vector3d& gpsA,
                                  const Vector3d& gpsB);
    };
}

// ============================================================
// 大气参数模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Atmosphere {
        // 大气参数结构
        struct Parameters {
            double pressure;       // 气压 (Pa)
            double density;        // 空气密度 (kg/m³)
            double gravity;        // 重力加速度 (m/s²)
            double sound_speed;    // 声速 (m/s)
            double temperature;    // 温度 (K)
        };

        // 根据高度获取完整大气参数
        static Parameters GetParameters(double height_meters);

        // ===== 单项计算函数 =====

        // 重力加速度 (m/s²)
        static double Gravity(double height_meters);

        // 声速 (m/s)
        static double SoundSpeed(double height_meters);

        // 空气密度 (kg/m³)
        static double Density(double height_meters);

        // 气压 (Pa)
        static double Pressure(double height_meters);

        // 温度 (K)
        static double Temperature(double height_meters);

        // ===== 速度转换 =====

        // 根据马赫数和高度计算速度
        static double VelocityFromMach(double mach, double height_meters);

        // 根据速度和高度计算马赫数
        static double MachFromVelocity(double velocity, double height_meters);

        // ===== 动压计算 =====

        // 动压 q = 0.5 * rho * V²
        static double DynamicPressure(double velocity, double height_meters);
    };
}

// ============================================================
// 随机数生成模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Random {
        // 设置随机种子（默认使用当前时间）
        static void Seed(unsigned int seed = 0);

        // ===== 均匀分布 =====

        // [0, 1] 均匀分布
        static double Uniform01();

        // [a, b] 均匀分布
        static double Uniform(double a, double b);

        // 整数均匀分布 [a, b]
        static int UniformInt(int a, int b);

        // ===== 正态分布 =====

        // 标准正态分布 N(0, 1)
        static double Normal01();

        // 正态分布 N(mu, sigma²)
        static double Normal(double mu, double sigma);

        // ===== 其他分布 =====

        // 指数分布
        static double Exponential(double lambda);

        // 韦伯分布
        static double Weibull(double shape, double scale);
    };
}

// ============================================================
// 文件 I/O 模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API FileIO {
        // ===== 读取数据 =====

        // 读取文本文件到 Eigen 矩阵（自动处理空格和逗号分隔）
        static MatrixXd ReadMatrix(const std::string& filename);

        // 读取文本文件到二维 vector
        static std::vector<std::vector<double>> ReadTable(const std::string& filename);

        // 读取单列数据
        static std::vector<double> ReadColumn(const std::string& filename,
                                             int column_index = 0);

        // ===== 写入数据 =====

        // 将 Eigen 矩阵写入文件
        static bool WriteMatrix(const std::string& filename,
                               const MatrixXd& matrix,
                               bool scientific = false,
                               int precision = 6);

        // 将 vector 写入文件
        static bool WriteVector(const std::string& filename,
                               const std::vector<double>& data,
                               bool scientific = false,
                               int precision = 6);

        // ===== 文件信息 =====

        // 统计文件行数
        static int CountLines(const std::string& filename);

        // 统计文件列数（第一行）
        static int CountColumns(const std::string& filename);

        // ===== 辅助功能 =====

        // 数字转字符串
        static std::string ToString(int value);
        static std::string ToString(double value, int precision = 6);

        // 判断文件是否存在
        static bool Exists(const std::string& filename);
    };
}

// ============================================================
// 数值计算模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Numerical {
        // ===== 数值积分 =====

        // 龙格-库塔 4 阶积分
        // f: 微分方程函数 dy/dt = f(t, y)
        // t0: 初始时间, y0: 初始状态, h: 步长, steps: 积分步数
        using OdeFunction = std::function<VectorXd(double, const VectorXd&)>;

        static VectorXd RungeKutta4(const OdeFunction& f,
                                          double t0,
                                          const VectorXd& y0,
                                          double h,
                                          int steps);

        // 自适应步长龙格-库塔
        static VectorXd RungeKutta45(const OdeFunction& f,
                                           double t0,
                                           const VectorXd& y0,
                                           double t_end,
                                           double tolerance = 1e-6);

        // ===== 根查找 =====

        // 二分法求根
        static double Bisection(const std::function<double(double)>& f,
                               double a, double b,
                               double tol = 1e-10);

        // 牛顿法求根
        static double Newton(const std::function<double(double)>& f,
                           const std::function<double(double)>& df,
                           double x0,
                           double tol = 1e-10,
                           int max_iter = 100);

        // ===== 数值微分 =====

        // 中心差分求导数
        static double Derivative(const std::function<double(double)>& f,
                                double x,
                                double h = 1e-6);
    };
}

// ============================================================
// 几何计算模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Geometry {
        // 二维点结构
        struct Point2D {
            double x, y;
            Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
        };

        // ===== 点与多边形关系 =====

        // 判断点是否在三角形内
        static bool IsPointInTriangle(const Point2D& point,
                                     const Point2D& a,
                                     const Point2D& b,
                                     const Point2D& c);

        // 判断点是否在多边形内
        static bool IsPointInPolygon(const Point2D& point,
                                    const std::vector<Point2D>& polygon);

        // ===== 距离计算 =====

        // 点到线段的距离
        static double DistanceToLineSegment(const Point2D& point,
                                          const Point2D& line_start,
                                          const Point2D& line_end);

        // 两点间距离
        static inline double Distance(const Point2D& p1, const Point2D& p2) {
            return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
        }

        // ===== 三维几何 =====

        // 三点确定的平面法向量
        static Vector3d PlaneNormal(const Vector3d& p1,
                                          const Vector3d& p2,
                                          const Vector3d& p3);

        // 点到平面的距离
        static double DistanceToPlane(const Vector3d& point,
                                     const Vector3d& plane_point,
                                     const Vector3d& plane_normal);
    };
}

// ============================================================
// 时间和日期工具
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Time {
        // GPS 时间戳结构
        struct GpsTime {
            int week;          // GPS 周
            double seconds;    // 周内秒
        };

        // Unix 时间戳 (秒)
        static double GetUnixTimestamp();

        // Unix 时间 -> GPS 时间
        static GpsTime UnixToGpsTime(double unix_time);

        // GPS 时间 -> Unix 时间
        static double GpsTimeToUnix(const GpsTime& gps_time);

        // UTC -> GPS 时间秒
        static double UtcToGpsSeconds(int year, int month, int day,
                                     int hour, int minute, double second);

        // 格林威治恒星时
        static double GreenwhichSiderealTime(double mjd);
    };
}

// ============================================================
// 单位转换模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Units {
        // ===== 距离 =====
        static constexpr double M_TO_FT = 3.2808399;
        static constexpr double FT_TO_M = 1.0 / M_TO_FT;
        static constexpr double KM_TO_M = 1000.0;
        static constexpr double NM_TO_M = 1852.0;  // 海里

        // ===== 速度 =====
        static constexpr double KNOTS_TO_MS = 0.514444444;
        static constexpr double MS_TO_KNOTS = 1.0 / KNOTS_TO_MS;
        static constexpr double KMH_TO_MS = 0.277777778;
        static constexpr double MS_TO_KMH = 1.0 / KMH_TO_MS;

        // ===== 角度 =====
        static constexpr double DEG_TO_RAD = Constants::DEG_TO_RAD;
        static constexpr double RAD_TO_DEG = Constants::RAD_TO_DEG;
        static constexpr double MRAD_TO_RAD = 0.001;
        static constexpr double RAD_TO_MRAD = 1000.0;

        // ===== 质量 =====
        static constexpr double KG_TO_LB = 2.20462262;
        static constexpr double LB_TO_KG = 1.0 / KG_TO_LB;

        // ===== 力/推力 =====
        static constexpr double N_TO_LBF = 0.224808943;
        static constexpr double LBF_TO_N = 1.0 / N_TO_LBF;
        static constexpr double KN_TO_N = 1000.0;

        // ===== 压力 =====
        static constexpr double PA_TO_PSI = 0.000145037738;
        static constexpr double PSI_TO_PA = 1.0 / PA_TO_PSI;
        static constexpr double PA_TO_BAR = 1e-5;
        static constexpr double BAR_TO_PA = 1e5;
        static constexpr double ATM_TO_PA = 101325.0;

        // ===== 温度 =====
        static inline double CelsiusToKelvin(double c) { return c + 273.15; }
        static inline double KelvinToCelsius(double k) { return k - 273.15; }
        static inline double FahrenheitToKelvin(double f) {
            return (f - 32.0) * 5.0 / 9.0 + 273.15;
        }
    };
}

// ============================================================
// 矩阵运算工具（Eigen 扩展）
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API MatrixUtils {
        // 矩阵乘法（C风格数组接口）
        static void Multiply(const double A[3][3], const double B[3], double C[3]);
        static void Multiply3x3(const double A[3][3], const double B[3][3], double C[3][3]);

        // 矩阵转置
        static void Transpose(const double A[3][3], double B[3][3]);

        // 向量外积
        static Matrix3d OuterProduct(const Vector3d& a,
                                           const Vector3d& b);

        // 斜对称矩阵（用于叉乘）
        static Matrix3d SkewSymmetric(const Vector3d& v);

        // 四元数 -> 旋转矩阵
        static Matrix3d QuaternionToMatrix(const Vector4d& q);

        // 旋转矩阵 -> 欧拉角
        static Vector3d MatrixToEuler(const Matrix3d& R);

        // 欧拉角 -> 旋转矩阵
        static Matrix3d EulerToMatrix(double roll, double pitch, double yaw);
    };
}

// ============================================================
// 仿真实用工具
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Simulation {
        // 性能计时器
        class Timer {
        public:
            Timer();
            ~Timer();
            void Start();
            void Stop();
            double ElapsedSeconds() const;
            double ElapsedMilliseconds() const;
        private:
            class Impl;
            std::unique_ptr<Impl> impl_;
        };

        // 日志级别
        enum class LogLevel {
            Debug,
            Info,
            Warning,
            Error,
            Fatal
        };

        // 简单日志功能
        class Logger {
        public:
            static void Log(LogLevel level, const std::string& message);
            static void Debug(const std::string& message);
            static void Info(const std::string& message);
            static void Warning(const std::string& message);
            static void Error(const std::string& message);

            static void SetLogLevel(LogLevel level);
            static void SetOutputFile(const std::string& filename);
        };
    };
}

// ============================================================
// 版本信息
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Version {
        static constexpr int MAJOR = 2;
        static constexpr int MINOR = 0;
        static constexpr int PATCH = 0;

        static std::string ToString() {
            return std::to_string(MAJOR) + "." +
                   std::to_string(MINOR) + "." +
                   std::to_string(PATCH);
        }
    };
}
