// ============================================================
// SimTools v2.0 - 单文件版本示例程序
// ============================================================
//
// 使用方法：
// 1. 将此文件和 single_file/SimTools_v2.h 复制到你的项目目录
// 2. 在你的代码中包含这两个文件
// 3. 编译运行
//
// ============================================================

// 定义标志以禁用 SimTools_v2_SingleFile.cpp 中的 main()
#define SIMTOOLS_SINGLE_FILE_NO_MAIN

#include "SimTools_v2.h"
#include <iostream>
#include <iomanip>

// 包含单文件实现
#include "SimTools_v2_SingleFile.cpp"

using namespace SimTools;

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  SimTools v2.0 - 单文件版本演示" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Version: " << Version::ToString() << std::endl;
    std::cout << std::endl;

    // ===== 示例 1：坐标转换 =====
    std::cout << "【示例 1】坐标转换" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    Coordinate::Vector3 beijing(116.3974, 39.9093, 100.0);
    std::cout << "北京坐标 (GPS): " << beijing << std::endl;

    Coordinate::Vector3 ecef = Coordinate::GpsToEcef(beijing);
    std::cout << "北京坐标 (ECEF): " << ecef << std::endl;

    Coordinate::Vector3 back = Coordinate::EcefToGps(ecef);
    std::cout << "转回 GPS: " << back << std::endl;
    std::cout << std::endl;

    // ===== 示例 2：地理计算 =====
    std::cout << "【示例 2】地理计算" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    Coordinate::Vector3 shanghai(121.4737, 31.2304, 0.0);
    double distance = Geodesy::GreatCircleDistance(
        beijing[0], beijing[1],
        shanghai[0], shanghai[1]
    );
    std::cout << "北京到上海距离: " << (distance / 1000.0) << " km" << std::endl;

    double azimuth = Geodesy::Azimuth(beijing, shanghai);
    std::cout << "方位角: " << azimuth << " 度" << std::endl;
    std::cout << std::endl;

    // ===== 示例 3：大气参数 =====
    std::cout << "【示例 3】大气参数查询" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    double altitude = 10000.0;  // 10 km
    auto air = Atmosphere::GetParameters(altitude);

    std::cout << "高度: " << altitude << " m" << std::endl;
    std::cout << "温度: " << air.temperature << " K ("
              << (air.temperature - 273.15) << " °C)" << std::endl;
    std::cout << "气压: " << air.pressure << " Pa" << std::endl;
    std::cout << "密度: " << air.density << " kg/m³" << std::endl;
    std::cout << "声速: " << air.sound_speed << " m/s" << std::endl;
    std::cout << std::endl;

    // ===== 示例 4：插值计算 =====
    std::cout << "【示例 4】数据插值" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::vector<double> x = {0, 1, 2, 3, 4, 5};
    std::vector<double> y = {0, 1, 8, 27, 64, 125};  // y = x³

    double x_query = 2.5;
    double y_interp = Interpolation::Linear(x_query, x, y);
    double y_lagrange = Interpolation::Lagrange7(x_query, x, y);

    std::cout << "函数: y = x³" << std::endl;
    std::cout << "查询点 x = " << x_query << std::endl;
    std::cout << "线性插值: " << y_interp << std::endl;
    std::cout << "拉格朗日插值: " << y_lagrange << std::endl;
    std::cout << "真实值: " << (x_query * x_query * x_query) << std::endl;
    std::cout << std::endl;

    // ===== 示例 5：数学工具 =====
    std::cout << "【示例 5】数学工具" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    double angle = 370.0;
    std::cout << "角度规范化测试:" << std::endl;
    std::cout << "  原始角度: " << angle << "°" << std::endl;
    std::cout << "  规范化到 [-180, 180]: " << Math::Regulate180(angle) << "°" << std::endl;
    std::cout << "  规范化到 [0, 360]: " << Math::Regulate360(angle) << "°" << std::endl;
    std::cout << std::endl;

    // ===== 示例 6：随机数 =====
    std::cout << "【示例 6】随机数生成" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    Random::Seed(42);
    std::cout << "固定种子的随机数:" << std::endl;
    for (int i = 0; i < 5; i++) {
        std::cout << "  均匀分布 [0, 1): " << Random::Uniform01() << std::endl;
    }
    std::cout << std::endl;

    // ===== 示例 7：单位转换 =====
    std::cout << "【示例 7】单位转换" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    double knots = 100.0;
    double ms = knots * Units::KNOTS_TO_MS;
    std::cout << knots << " knots = " << ms << " m/s = "
              << (ms * 3.6) << " km/h" << std::endl;
    std::cout << std::endl;

    std::cout << "========================================" << std::endl;
    std::cout << "  单文件版本演示完成！" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
