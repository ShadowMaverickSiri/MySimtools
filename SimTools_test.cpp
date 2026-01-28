// ============================================================
// SimTools v2.0 - 单元测试示例
// ============================================================

#include "SimTools_v2.h"
#include <iostream>
#include <cassert>
#include <iomanip>

using namespace SimTools;

// 测试辅助宏
#define TEST(name) \
    std::cout << "Running test: " << #name << "..."; \
    try { \
        name(); \
        std::cout << " PASSED" << std::endl; \
    } catch (const std::exception& e) { \
        std::cout << " FAILED: " << e.what() << std::endl; \
        return 1; \
    }

#define ASSERT_NEAR(a, b, tol) \
    if (std::abs((a) - (b)) > (tol)) { \
        throw std::runtime_error("Assertion failed: " #a " != " #b); \
    }

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        throw std::runtime_error("Assertion failed: " #cond); \
    }

#define ASSERT_FALSE(cond) \
    if ((cond)) { \
        throw std::runtime_error("Assertion failed: !" #cond); \
    }

// ============================================================
// 测试用例
// ============================================================

void Test_Math() {
    // 测试符号函数
    ASSERT_TRUE(Math::Sign(5.0) == 1);
    ASSERT_TRUE(Math::Sign(-5.0) == -1);
    ASSERT_TRUE(Math::Sign(0.0) == 0);

    // 测试最大最小值
    ASSERT_TRUE(Math::Max(3.0, 7.0) == 7.0);
    ASSERT_TRUE(Math::Min(3.0, 7.0) == 3.0);

    // 测试Clamp
    ASSERT_TRUE(Math::Clamp(5.0, 0.0, 10.0) == 5.0);
    ASSERT_TRUE(Math::Clamp(-5.0, 0.0, 10.0) == 0.0);
    ASSERT_TRUE(Math::Clamp(15.0, 0.0, 10.0) == 10.0);

    // 测试角度规范化
    ASSERT_NEAR(Math::Regulate180(370.0), 10.0, 1e-10);
    ASSERT_NEAR(Math::Regulate180(-190.0), 170.0, 1e-10);
    ASSERT_NEAR(Math::Regulate360(400.0), 40.0, 1e-10);

    // 测试向量范数
    Vector3d v(3.0, 4.0, 0.0);
    ASSERT_NEAR(Math::Norm2(v), 5.0, 1e-10);

    // 测试归一化
    Vector3d v_norm = Math::Normalize(v);
    ASSERT_NEAR(Math::Norm2(v_norm), 1.0, 1e-10);
}

void Test_Interpolation() {
    // 线性插值测试
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> y = {0.0, 2.0, 4.0, 6.0, 8.0};  // y = 2x

    double y_interp = Interpolation::Linear(2.5, x, y);
    ASSERT_NEAR(y_interp, 5.0, 1e-10);

    // 边界测试
    y_interp = Interpolation::Linear(-1.0, x, y);
    ASSERT_NEAR(y_interp, 0.0, 1e-10);

    y_interp = Interpolation::Linear(5.0, x, y);
    ASSERT_NEAR(y_interp, 8.0, 1e-10);

    // 拉格朗日插值测试
    std::vector<double> x2 = {0.0, 1.0, 2.0, 3.0};
    std::vector<double> y2 = {0.0, 1.0, 4.0, 9.0};  // y = x^2

    double y_lagrange = Interpolation::Lagrange7(1.5, x2, y2);
    ASSERT_NEAR(y_lagrange, 2.25, 0.01);
}

void Test_Coordinate() {
    // GPS 坐标
    Coordinate::Vector3 beijing_gps(116.3974, 39.9093, 100.0);

    // GPS -> ECEF -> GPS 往返转换
    Coordinate::Vector3 ecef = Coordinate::GpsToEcef(beijing_gps);
    Coordinate::Vector3 gps_back = Coordinate::EcefToGps(ecef);

    // 检查往返转换误差
    ASSERT_NEAR(gps_back[0], beijing_gps[0], 1e-6);
    ASSERT_NEAR(gps_back[1], beijing_gps[1], 1e-6);
    ASSERT_NEAR(gps_back[2], beijing_gps[2], 1e-3);

    // 牛顿法转换测试
    Coordinate::Vector3 gps_newton = Coordinate::EcefToGpsNewton(ecef);
    ASSERT_NEAR(gps_newton[0], beijing_gps[0], 1e-8);
    ASSERT_NEAR(gps_newton[1], beijing_gps[1], 1e-8);
    ASSERT_NEAR(gps_newton[2], beijing_gps[2], 1e-5);

    // NED 坐标转换测试
    Coordinate::Vector3 shanghai_gps(121.4737, 31.2304, 0.0);
    Coordinate::Vector3 shanghai_ned = Coordinate::EcefToNed(
        Coordinate::GpsToEcef(shanghai_gps),
        beijing_gps
    );

    // 检查 NED 坐标合理性（上海在北京的东南方向）
    // 上海纬度更低（在南边），所以北向为负
    ASSERT_TRUE(shanghai_ned[0] < 0);  // 北向（南为负）
    ASSERT_TRUE(shanghai_ned[1] > 0);  // 东向（东为正）
}

void Test_Geodesy() {
    // 北京和上海的坐标
    double lon1 = 116.3974, lat1 = 39.9093;
    double lon2 = 121.4737, lat2 = 31.2304;

    // 大圆距离测试
    double distance = Geodesy::GreatCircleDistance(lon1, lat1, lon2, lat2);
    ASSERT_NEAR(distance / 1000.0, 1067.0, 20.0);  // 约 1067 km

    // 方位角测试
    double azimuth = Geodesy::Azimuth(
        Coordinate::Vector3(lon1, lat1, 0.0),
        Coordinate::Vector3(lon2, lat2, 0.0)
    );
    ASSERT_TRUE(azimuth > 140.0 && azimuth < 170.0);  // 东南偏南方向（约153度）
}

void Test_Atmosphere() {
    // 海平面参数测试
    auto air_sea = Atmosphere::GetParameters(0.0);
    ASSERT_NEAR(air_sea.temperature, 288.15, 0.1);
    ASSERT_NEAR(air_sea.pressure, 101325.0, 100.0);
    ASSERT_NEAR(air_sea.gravity, 9.81, 0.01);
    ASSERT_NEAR(air_sea.sound_speed, 340.0, 5.0);

    // 10 km 高度测试
    auto air_10km = Atmosphere::GetParameters(10000.0);
    ASSERT_TRUE(air_10km.temperature < 250.0);  // 温度随高度降低
    ASSERT_TRUE(air_10km.pressure < 30000.0);   // 气压随高度降低
    ASSERT_TRUE(air_10km.density < 0.5);        // 密度随高度降低

    // 马赫数转换测试
    double velocity = Atmosphere::VelocityFromMach(1.0, 10000.0);
    double mach = Atmosphere::MachFromVelocity(velocity, 10000.0);
    ASSERT_NEAR(mach, 1.0, 0.01);
}

void Test_Random() {
    Random::Seed(42);  // 使用固定种子保证可重复性

    // 测试均匀分布
    double u = Random::Uniform01();
    ASSERT_TRUE(u >= 0.0 && u < 1.0);

    double u_ab = Random::Uniform(10.0, 20.0);
    ASSERT_TRUE(u_ab >= 10.0 && u_ab < 20.0);

    // 测试正态分布
    double n = Random::Normal01();
    ASSERT_TRUE(n > -10.0 && n < 10.0);  // 99.7% 在 3σ 范围内

    double n_custom = Random::Normal(100.0, 15.0);
    ASSERT_TRUE(n_custom > 50.0 && n_custom < 150.0);
}

void Test_Geometry() {
    // 测试点在三角形内
    Geometry::Point2D p(2.0, 2.0);
    Geometry::Point2D a(0.0, 0.0);
    Geometry::Point2D b(4.0, 0.0);
    Geometry::Point2D c(2.0, 4.0);

    ASSERT_TRUE(Geometry::IsPointInTriangle(p, a, b, c));

    // 测试点在三角形外
    Geometry::Point2D p_out(5.0, 5.0);
    ASSERT_FALSE(Geometry::IsPointInTriangle(p_out, a, b, c));

    // 测试点在多边形内
    std::vector<Geometry::Point2D> square = {
        {0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}
    };
    Geometry::Point2D p_inside(2.0, 2.0);
    Geometry::Point2D p_outside(5.0, 5.0);

    ASSERT_TRUE(Geometry::IsPointInPolygon(p_inside, square));
    ASSERT_FALSE(Geometry::IsPointInPolygon(p_outside, square));

    // 测试距离计算
    double dist = Geometry::Distance(a, b);
    ASSERT_NEAR(dist, 4.0, 1e-10);
}

void Test_MatrixUtils() {
    // 测试欧拉角转旋转矩阵
    double roll = 0.1, pitch = 0.2, yaw = 0.3;
    Matrix3d R = MatrixUtils::EulerToMatrix(roll, pitch, yaw);

    // 检查正交性
    ASSERT_NEAR((R * R.transpose()).trace(), 3.0, 1e-10);

    // 测试往返转换
    Vector3d euler_back = MatrixUtils::MatrixToEuler(R);
    ASSERT_NEAR(euler_back[0], roll, 1e-10);
    ASSERT_NEAR(euler_back[1], pitch, 1e-10);
    ASSERT_NEAR(euler_back[2], yaw, 1e-10);

    // 测试斜对称矩阵
    Vector3d v(1.0, 2.0, 3.0);
    Matrix3d S = MatrixUtils::SkewSymmetric(v);

    // 检查 S^T = -S
    ASSERT_NEAR((S + S.transpose()).norm(), 0.0, 1e-10);
}

void Test_Units() {
    // 测试单位转换
    double knots = 100.0;
    double ms = knots * Units::KNOTS_TO_MS;
    ASSERT_NEAR(ms, 51.44, 0.1);

    double km = 5.0;
    double nm = km * 1000.0 / Units::NM_TO_M;
    ASSERT_NEAR(nm, 2.7, 0.1);

    // 测试温度转换
    double c = 25.0;
    double k = Units::CelsiusToKelvin(c);
    ASSERT_NEAR(k, 298.15, 0.01);

    double f = 77.0;
    double k_f = Units::FahrenheitToKelvin(f);
    ASSERT_NEAR(k_f, 298.15, 0.5);
}

void Test_Numerical() {
    // 测试求根
    auto f = [](double x) { return x * x - 4.0; };
    auto df = [](double x) { return 2.0 * x; };

    double root = Numerical::Newton(f, df, 3.0);
    ASSERT_NEAR(root, 2.0, 1e-6);

    root = Numerical::Bisection(f, 0.0, 5.0);
    ASSERT_NEAR(root, 2.0, 1e-3);

    // 测试数值微分
    auto g = [](double x) { return x * x; };
    double deriv = Numerical::Derivative(g, 2.0);
    ASSERT_NEAR(deriv, 4.0, 1e-4);
}

void Test_FileIO() {
    // 测试文件写入
    std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};
    bool success = FileIO::WriteVector("test_data.txt", data);
    ASSERT_TRUE(success);

    // 测试文件读取
    std::vector<double> data_read = FileIO::ReadColumn("test_data.txt", 0);
    ASSERT_TRUE(data_read.size() == data.size());

    for (size_t i = 0; i < data.size(); ++i) {
        ASSERT_NEAR(data_read[i], data[i], 1e-10);
    }

    // 测试行数统计
    int lines = FileIO::CountLines("test_data.txt");
    ASSERT_TRUE(lines == 5);

    // 测试文件存在性
    ASSERT_TRUE(FileIO::Exists("test_data.txt"));
}

// ============================================================
// 主测试函数
// ============================================================

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  SimTools v2.0 - 单元测试" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // 运行所有测试
    TEST(Test_Math);
    TEST(Test_Interpolation);
    TEST(Test_Coordinate);
    TEST(Test_Geodesy);
    TEST(Test_Atmosphere);
    TEST(Test_Random);
    TEST(Test_Geometry);
    TEST(Test_MatrixUtils);
    TEST(Test_Units);
    TEST(Test_Numerical);
    TEST(Test_FileIO);

    std::cout << "========================================" << std::endl;
    std::cout << "  所有测试通过！" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
