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

        // 几何高度 -> 位势高度
        double h = std::max(0.0, std::min(height_meters, 90000.0));
        const double geopotential_radius = 6356766.0;
        double H = geopotential_radius * h / (geopotential_radius + h);

        // 1976 标准大气分层参数
        double base_height;
        double base_temperature;
        double lapse_rate;
        double base_pressure;
        if (H <= 11000) {
            base_height = 0.0;
            base_temperature = 288.15;
            lapse_rate = -0.0065;
            base_pressure = 101325.0;
        } else if (H <= 20000) {
            base_height = 11000.0;
            base_temperature = 216.65;
            lapse_rate = 0.0;
            base_pressure = 22632.0;
        } else if (H <= 32000) {
            base_height = 20000.0;
            base_temperature = 216.65;
            lapse_rate = 0.001;
            base_pressure = 5474.870;
        } else if (H <= 47000) {
            base_height = 32000.0;
            base_temperature = 228.65;
            lapse_rate = 0.0028;
            base_pressure = 868.015;
        } else if (H <= 51000) {
            base_height = 47000.0;
            base_temperature = 270.65;
            lapse_rate = 0.0;
            base_pressure = 110.906;
        } else {
            base_height = 51000.0;
            base_temperature = 270.65;
            lapse_rate = -0.0028;
            base_pressure = 75.944;
        }

        double delta_height = H - base_height;
        params.temperature = base_temperature + lapse_rate * delta_height;
        if (lapse_rate == 0.0) {
            params.pressure = base_pressure * std::exp(
                -Constants::GRAVITY_SEA_LEVEL * delta_height /
                (Constants::GAS_CONSTANT_AIR * params.temperature));
        } else {
            params.pressure = base_pressure * std::pow(
                1.0 + lapse_rate * delta_height / base_temperature,
                -Constants::GRAVITY_SEA_LEVEL /
                (Constants::GAS_CONSTANT_AIR * lapse_rate));
        }

        // 计算其他参数
        params.gravity = Constants::GRAVITY_SEA_LEVEL *
                        std::pow(geopotential_radius / (geopotential_radius + h), 2);

        params.density = params.pressure / (Constants::GAS_CONSTANT_AIR * params.temperature);

        params.sound_speed = std::sqrt(1.4 * Constants::GAS_CONSTANT_AIR * params.temperature);

        return params;
    }

    // ============================================================
    // 单项计算函数
    // ============================================================

    double Atmosphere::Gravity(double height_meters) {
        double h = std::max(0.0, height_meters);
        const double geopotential_radius = 6356766.0;
        return Constants::GRAVITY_SEA_LEVEL *
               std::pow(geopotential_radius / (geopotential_radius + h), 2);
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
