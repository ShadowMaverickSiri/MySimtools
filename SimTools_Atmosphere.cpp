// ============================================================
// SimTools v2.0 - 大气参数模块实现
// ============================================================

#include "SimTools_v2.h"
#include <cmath>

namespace SimTools {

    // ============================================================
    // 完整大气参数获取
    // ============================================================

    Atmosphere::Parameters Atmosphere::GetParameters(double height_meters) {
        Parameters params;

        // 限制高度范围 [0, 90000] 米
        double h = std::max(0.0, std::min(height_meters, 90000.0));

        // 简化的国际标准大气模型（ISA）
        // 实际应用中应使用 1976 标准大气模型

        if (h <= 11000) {
            // 对流层 (0 - 11 km)
            // 温度递减率: -6.5 K/km
            params.temperature = 288.15 - 0.0065 * h;
            params.pressure = 101325.0 * std::pow(1 - 0.0065 * h / 288.15, 5.25588);
        } else if (h <= 20000) {
            // 对流层顶 / 平流层下层 (11 - 20 km)
            // 温度恒定: 216.65 K
            params.temperature = 216.65;
            params.pressure = 22632.0 * std::exp(-(h - 11000) / 6341.62);
        } else if (h <= 32000) {
            // 平流层中层 (20 - 32 km)
            // 温度递增率: +1.0 K/km
            params.temperature = 216.65 + 0.001 * (h - 20000);
            params.pressure = 5474.87 * std::pow(1 + 0.001 * (h - 20000) / 216.65, -34.163);
        } else if (h <= 47000) {
            // 平流层上层 (32 - 47 km)
            // 温度递增率: +2.8 K/km
            params.temperature = 228.65 + 0.0028 * (h - 32000);
            params.pressure = 868.015 * std::pow(1 + 0.0028 * (h - 32000) / 228.65, -12.201);
        } else if (h <= 51000) {
            // 平流层顶 (47 - 51 km)
            // 温度恒定: 270.65 K
            params.temperature = 270.65;
            params.pressure = 110.906 * std::exp(-(h - 47000) / 7922.0);
        } else {
            // 中间层 (51 - 90 km)
            // 温度递减率: -2.8 K/km
            params.temperature = 270.65 - 0.0028 * (h - 51000);
            params.pressure = 75.944 * std::pow(1 - 0.0028 * (h - 51000) / 270.65, 12.201);
        }

        // 计算其他参数
        params.gravity = Constants::GRAVITY_SEA_LEVEL *
                        std::pow(Constants::EARTH_SEMIMAJOR / (Constants::EARTH_SEMIMAJOR + h), 2);

        params.density = params.pressure / (Constants::GAS_CONSTANT_AIR * params.temperature);

        params.sound_speed = std::sqrt(1.4 * Constants::GAS_CONSTANT_AIR * params.temperature);

        return params;
    }

    // ============================================================
    // 单项计算函数
    // ============================================================

    double Atmosphere::Gravity(double height_meters) {
        double h = std::max(0.0, height_meters);
        return Constants::GRAVITY_SEA_LEVEL *
               std::pow(Constants::EARTH_SEMIMAJOR / (Constants::EARTH_SEMIMAJOR + h), 2);
    }

    double Atmosphere::SoundSpeed(double height_meters) {
        return GetParameters(height_meters).sound_speed;
    }

    double Atmosphere::Density(double height_meters) {
        return GetParameters(height_meters).density;
    }

    double Atmosphere::Pressure(double height_meters) {
        return GetParameters(height_meters).pressure;
    }

    double Atmosphere::Temperature(double height_meters) {
        return GetParameters(height_meters).temperature;
    }

    // ============================================================
    // 速度转换
    // ============================================================

    double Atmosphere::VelocityFromMach(double mach, double height_meters) {
        double sound_speed = SoundSpeed(height_meters);
        return mach * sound_speed;
    }

    double Atmosphere::MachFromVelocity(double velocity, double height_meters) {
        double sound_speed = SoundSpeed(height_meters);
        if (sound_speed < 1e-10) {
            return 0.0;
        }
        return velocity / sound_speed;
    }

    // ============================================================
    // 动压计算
    // ============================================================

    double Atmosphere::DynamicPressure(double velocity, double height_meters) {
        double rho = Density(height_meters);
        return 0.5 * rho * velocity * velocity;
    }

} // namespace SimTools
