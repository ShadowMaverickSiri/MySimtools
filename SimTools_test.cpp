// ============================================================
// SimTools v2.0 - 测试程序
// ============================================================

#include "SimTools_v2.h"
#include <iostream>
#include <iomanip>
#include <cmath>

#ifdef _WIN32
    #include <windows.h>  // 用于设置控制台编码
#endif

using namespace SimTools;

// 测试辅助函数
void PrintTestResult(const std::string& test_name, bool passed) {
    std::cout << "  " << test_name << ": ";
    if (passed) {
        std::cout << "✓ PASSED" << std::endl;
    } else {
        std::cout << "✗ FAILED" << std::endl;
    }
}

bool IsNear(double a, double b, double tol = 1e-10) {
    return std::abs(a - b) < tol;
}

// ============================================================
// 测试函数
// ============================================================

void Test_Math() {
    std::cout << "\n【测试 1】数学工具模块" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    bool all_passed = true;

    // 测试符号函数
    bool test1 = (Math::Sign(5.0) == 1) &&
                 (Math::Sign(-5.0) == -1) &&
                 (Math::Sign(0.0) == 0);
    PrintTestResult("Sign函数", test1);
    all_passed &= test1;

    // 测试最大最小值
    bool test2 = (Math::Max(3.0, 7.0) == 7.0) &&
                 (Math::Min(3.0, 7.0) == 3.0);
    PrintTestResult("Max/Min函数", test2);
    all_passed &= test2;

    // 测试Clamp
    bool test3 = (Math::Clamp(5.0, 0.0, 10.0) == 5.0) &&
                 (Math::Clamp(-5.0, 0.0, 10.0) == 0.0) &&
                 (Math::Clamp(15.0, 0.0, 10.0) == 10.0);
    PrintTestResult("Clamp函数", test3);
    all_passed &= test3;

    // 测试角度规范化
    bool test4 = IsNear(Math::Regulate180(370.0), 10.0) &&
                 IsNear(Math::Regulate180(-190.0), 170.0) &&
                 IsNear(Math::Regulate360(400.0), 40.0);
    PrintTestResult("角度规范化", test4);
    all_passed &= test4;

    // 测试向量范数
    Vector3d v(3.0, 4.0, 0.0);
    bool test5 = IsNear(Math::Norm2(v), 5.0);
    PrintTestResult("向量范数", test5);
    all_passed &= test5;

    std::cout << "测试结果: " << (all_passed ? "全部通过" : "部分失败") << std::endl;
}

void Test_Interpolation() {
    std::cout << "\n【测试 2】插值算法模块" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    bool all_passed = true;

    // 线性插值测试
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> y = {0.0, 2.0, 4.0, 6.0, 8.0};  // y = 2x

    double y_interp = Interpolation::Linear(2.5, x, y);
    bool test1 = IsNear(y_interp, 5.0);
    PrintTestResult("线性插值", test1);
    all_passed &= test1;

    // 边界测试
    double y_interp2 = Interpolation::Linear(-1.0, x, y);
    double y_interp3 = Interpolation::Linear(5.0, x, y);
    bool test2 = IsNear(y_interp2, 0.0) && IsNear(y_interp3, 8.0);
    PrintTestResult("边界插值", test2);
    all_passed &= test2;

    // 拉格朗日插值测试
    std::vector<double> x2 = {0.0, 1.0, 2.0, 3.0};
    std::vector<double> y2 = {0.0, 1.0, 4.0, 9.0};  // y = x^2

    double y_lagrange = Interpolation::Lagrange7(1.5, x2, y2);
    bool test3 = IsNear(y_lagrange, 2.25, 0.01);
    PrintTestResult("拉格朗日插值", test3);
    all_passed &= test3;

    std::cout << "测试结果: " << (all_passed ? "全部通过" : "部分失败") << std::endl;
}

void Test_Coordinate() {
    std::cout << "\n【测试 3】坐标转换模块" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    bool all_passed = true;

    // GPS 坐标
    Coordinate::Vector3 beijing_gps(116.3974, 39.9093, 100.0);

    // GPS -> ECEF -> GPS 往返转换
    Coordinate::Vector3 ecef = Coordinate::GpsToEcef(beijing_gps);
    Coordinate::Vector3 gps_back = Coordinate::EcefToGps(ecef);

    // 检查往返转换误差
    bool test1 = IsNear(gps_back[0], beijing_gps[0], 1e-6) &&
                 IsNear(gps_back[1], beijing_gps[1], 1e-6) &&
                 IsNear(gps_back[2], beijing_gps[2], 1e-3);
    PrintTestResult("GPS往返转换", test1);
    all_passed &= test1;

    // 牛顿法转换测试
    Coordinate::Vector3 gps_newton = Coordinate::EcefToGpsNewton(ecef);
    bool test2 = IsNear(gps_newton[0], beijing_gps[0], 1e-8) &&
                 IsNear(gps_newton[1], beijing_gps[1], 1e-8) &&
                 IsNear(gps_newton[2], beijing_gps[2], 1e-5);
    PrintTestResult("牛顿法转换", test2);
    all_passed &= test2;

    // NED 坐标转换测试
    Coordinate::Vector3 shanghai_gps(121.4737, 31.2304, 0.0);
    Coordinate::Vector3 shanghai_ned = Coordinate::EcefToNed(
        Coordinate::GpsToEcef(shanghai_gps),
        beijing_gps
    );

    // 检查 NED 坐标合理性（上海在北京的东南方向）
    bool test3 = (shanghai_ned[0] < 0) && (shanghai_ned[1] > 0);
    PrintTestResult("NED坐标转换", test3);
    all_passed &= test3;

    std::cout << "测试结果: " << (all_passed ? "全部通过" : "部分失败") << std::endl;
}

void Test_Geodesy() {
    std::cout << "\n【测试 4】地理计算模块" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    bool all_passed = true;

    // 北京和上海的坐标
    double lon1 = 116.3974, lat1 = 39.9093;
    double lon2 = 121.4737, lat2 = 31.2304;

    // 大圆距离测试
    double distance = Geodesy::GreatCircleDistance(lon1, lat1, lon2, lat2);
    bool test1 = IsNear(distance / 1000.0, 1067.0, 20.0);
    PrintTestResult("大圆距离", test1);
    all_passed &= test1;

    // 方位角测试
    double azimuth = Geodesy::Azimuth(
        Coordinate::Vector3(lon1, lat1, 0.0),
        Coordinate::Vector3(lon2, lat2, 0.0)
    );
    bool test2 = (azimuth > 140.0 && azimuth < 170.0);
    PrintTestResult("方位角计算", test2);
    all_passed &= test2;

    std::cout << "测试结果: " << (all_passed ? "全部通过" : "部分失败") << std::endl;
}

void Test_Atmosphere() {
    std::cout << "\n【测试 5】大气参数模块" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    bool all_passed = true;

    // 海平面参数测试
    auto air_sea = Atmosphere::GetParameters(0.0);
    bool test1 = IsNear(air_sea.temperature, 288.15, 0.1) &&
                 IsNear(air_sea.pressure, 101325.0, 100.0) &&
                 IsNear(air_sea.gravity, 9.81, 0.01) &&
                 IsNear(air_sea.sound_speed, 340.0, 5.0);
    PrintTestResult("海平面参数", test1);
    all_passed &= test1;

    // 10 km 高度测试
    auto air_10km = Atmosphere::GetParameters(10000.0);
    bool test2 = (air_10km.temperature < 250.0) &&
                 (air_10km.pressure < 30000.0) &&
                 (air_10km.density < 0.5);
    PrintTestResult("10km高度参数", test2);
    all_passed &= test2;

    // 马赫数转换测试
    double velocity = Atmosphere::VelocityFromMach(1.0, 10000.0);
    double mach = Atmosphere::MachFromVelocity(velocity, 10000.0);
    bool test3 = IsNear(mach, 1.0, 0.01);
    PrintTestResult("马赫数转换", test3);
    all_passed &= test3;

    std::cout << "测试结果: " << (all_passed ? "全部通过" : "部分失败") << std::endl;
}

void Test_Random() {
    std::cout << "\n【测试 6】随机数生成模块" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    bool all_passed = true;

    Random::Seed(42);  // 使用固定种子保证可重复性

    // 测试均匀分布
    bool test1 = true;
    for (int i = 0; i < 10; i++) {
        double u = Random::Uniform01();
        if (u < 0.0 || u >= 1.0) {
            test1 = false;
            break;
        }
    }
    PrintTestResult("均匀分布[0,1)", test1);
    all_passed &= test1;

    // 测试范围均匀分布
    bool test2 = true;
    for (int i = 0; i < 10; i++) {
        double u_ab = Random::Uniform(10.0, 20.0);
        if (u_ab < 10.0 || u_ab >= 20.0) {
            test2 = false;
            break;
        }
    }
    PrintTestResult("均匀分布[a,b]", test2);
    all_passed &= test2;

    // 测试正态分布（99.7%在3σ范围内）
    bool test3 = true;
    for (int i = 0; i < 10; i++) {
        double n = Random::Normal01();
        if (n < -10.0 || n > 10.0) {
            test3 = false;
            break;
        }
    }
    PrintTestResult("正态分布", test3);
    all_passed &= test3;

    std::cout << "测试结果: " << (all_passed ? "全部通过" : "部分失败") << std::endl;
}

void Test_Geometry() {
    std::cout << "\n【测试 7】几何计算模块" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    bool all_passed = true;

    // 测试点在三角形内
    Geometry::Point2D p(2.0, 2.0);
    Geometry::Point2D a(0.0, 0.0);
    Geometry::Point2D b(4.0, 0.0);
    Geometry::Point2D c(2.0, 4.0);

    bool test1 = Geometry::IsPointInTriangle(p, a, b, c);
    PrintTestResult("点在三角形内", test1);
    all_passed &= test1;

    // 测试点在三角形外
    Geometry::Point2D p_out(5.0, 5.0);
    bool test2 = !Geometry::IsPointInTriangle(p_out, a, b, c);
    PrintTestResult("点在三角形外", test2);
    all_passed &= test2;

    // 测试点在多边形内
    std::vector<Geometry::Point2D> square = {
        {0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}
    };
    Geometry::Point2D p_inside(2.0, 2.0);
    Geometry::Point2D p_outside(5.0, 5.0);

    bool test3 = Geometry::IsPointInPolygon(p_inside, square) &&
                 !Geometry::IsPointInPolygon(p_outside, square);
    PrintTestResult("点在多边形内/外", test3);
    all_passed &= test3;

    // 测试距离计算
    bool test4 = IsNear(Geometry::Distance(a, b), 4.0);
    PrintTestResult("距离计算", test4);
    all_passed &= test4;

    std::cout << "测试结果: " << (all_passed ? "全部通过" : "部分失败") << std::endl;
}

void Test_MatrixUtils() {
    std::cout << "\n【测试 8】矩阵工具模块" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    bool all_passed = true;

    // 测试欧拉角转旋转矩阵
    double roll = 0.1, pitch = 0.2, yaw = 0.3;
    Matrix3d R = MatrixUtils::EulerToMatrix(roll, pitch, yaw);

    // 检查正交性
    bool test1 = IsNear((R * R.transpose()).trace(), 3.0);
    PrintTestResult("旋转矩阵正交性", test1);
    all_passed &= test1;

    // 测试往返转换
    Vector3d euler_back = MatrixUtils::MatrixToEuler(R);
    bool test2 = IsNear(euler_back[0], roll) &&
                 IsNear(euler_back[1], pitch) &&
                 IsNear(euler_back[2], yaw);
    PrintTestResult("欧拉角往返转换", test2);
    all_passed &= test2;

    // 测试斜对称矩阵
    Vector3d v(1.0, 2.0, 3.0);
    Matrix3d S = MatrixUtils::SkewSymmetric(v);

    // 检查 S^T = -S
    bool test3 = IsNear((S + S.transpose()).norm(), 0.0);
    PrintTestResult("斜对称矩阵", test3);
    all_passed &= test3;

    std::cout << "测试结果: " << (all_passed ? "全部通过" : "部分失败") << std::endl;
}

void Test_Units() {
    std::cout << "\n【测试 9】单位转换模块" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    bool all_passed = true;

    // 测试速度单位转换
    double knots = 100.0;
    double ms = knots * Units::KNOTS_TO_MS;
    bool test1 = IsNear(ms, 51.44, 0.1);
    PrintTestResult("节到米/秒", test1);
    all_passed &= test1;

    // 测试距离单位转换
    double km = 5.0;
    double nm = km * 1000.0 / Units::NM_TO_M;
    bool test2 = IsNear(nm, 2.7, 0.1);
    PrintTestResult("千米到海里", test2);
    all_passed &= test2;

    // 测试温度转换
    double c = 25.0;
    double k = Units::CelsiusToKelvin(c);
    bool test3 = IsNear(k, 298.15, 0.01);
    PrintTestResult("摄氏度到开尔文", test3);
    all_passed &= test3;

    std::cout << "测试结果: " << (all_passed ? "全部通过" : "部分失败") << std::endl;
}

void Test_Numerical() {
    std::cout << "\n【测试 10】数值计算模块" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    bool all_passed = true;

    // 测试求根
    auto f = [](double x) { return x * x - 4.0; };
    auto df = [](double x) { return 2.0 * x; };

    double root_newton = Numerical::Newton(f, df, 3.0);
    bool test1 = IsNear(root_newton, 2.0, 1e-6);
    PrintTestResult("牛顿法求根", test1);
    all_passed &= test1;

    double root_bisection = Numerical::Bisection(f, 0.0, 5.0);
    bool test2 = IsNear(root_bisection, 2.0, 1e-3);
    PrintTestResult("二分法求根", test2);
    all_passed &= test2;

    // 测试数值微分
    auto g = [](double x) { return x * x; };
    double deriv = Numerical::Derivative(g, 2.0);
    bool test3 = IsNear(deriv, 4.0, 1e-4);
    PrintTestResult("数值微分", test3);
    all_passed &= test3;

    std::cout << "测试结果: " << (all_passed ? "全部通过" : "部分失败") << std::endl;
}

void Test_FileIO() {
    std::cout << "\n【测试 11】文件读写模块" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    bool all_passed = true;

    // 测试文件写入
    std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};
    bool test1 = FileIO::WriteVector("test_data.txt", data);
    PrintTestResult("写入文件", test1);
    all_passed &= test1;

    // 测试文件读取
    std::vector<double> data_read = FileIO::ReadColumn("test_data.txt", 0);
    bool test2 = (data_read.size() == data.size());
    PrintTestResult("读取文件", test2);
    all_passed &= test2;

    // 验证数据一致性
    bool test3 = true;
    for (size_t i = 0; i < data.size(); ++i) {
        if (!IsNear(data_read[i], data[i])) {
            test3 = false;
            break;
        }
    }
    PrintTestResult("数据一致性", test3);
    all_passed &= test3;

    // 测试行数统计
    int lines = FileIO::CountLines("test_data.txt");
    bool test4 = (lines == 5);
    PrintTestResult("行数统计", test4);
    all_passed &= test4;

    // 测试文件存在性
    bool test5 = FileIO::Exists("test_data.txt");
    PrintTestResult("文件存在检查", test5);
    all_passed &= test5;

    std::cout << "测试结果: " << (all_passed ? "全部通过" : "部分失败") << std::endl;
}

// ============================================================
// 主程序
// ============================================================

int main() {
    // ===== 设置控制台编码为 UTF-8（解决 Windows 中文乱码问题）=====
    #ifdef _WIN32
        SetConsoleOutputCP(65001);  // 设置控制台输出为 UTF-8
        SetConsoleCP(65001);        // 设置控制台输入为 UTF-8
    #endif
    // ============================================================

    std::cout << "========================================" << std::endl;
    std::cout << "  SimTools v2.0 - 测试程序" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Version: " << Version::ToString() << std::endl;
    std::cout << std::endl;

    // 运行所有测试
    Test_Math();
    Test_Interpolation();
    Test_Coordinate();
    Test_Geodesy();
    Test_Atmosphere();
    Test_Random();
    Test_Geometry();
    Test_MatrixUtils();
    Test_Units();
    Test_Numerical();
    Test_FileIO();

    std::cout << "\n========================================" << std::endl;
    std::cout << "  所有测试运行完成！" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
