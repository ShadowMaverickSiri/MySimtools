// ============================================================
// SimTools v2.0 - 坐标转换模块实现
// ============================================================

#include "SimTools_v2.h"
#include <cmath>

namespace SimTools {

    // ============================================================
    // GPS <-> ECEF 转换
    // ============================================================

    Coordinate::Vector3 Coordinate::GpsToEcef(const Vector3& gps) {
        double lon = gps[0] * Constants::DEG_TO_RAD;
        double lat = gps[1] * Constants::DEG_TO_RAD;
        double h = gps[2];

        double sinLat = std::sin(lat);
        double cosLat = std::cos(lat);
        double sinLon = std::sin(lon);
        double cosLon = std::cos(lon);

        // 卯酉圈曲率半径
        double e2 = Constants::ECCENTRICITY_FIRST * Constants::ECCENTRICITY_FIRST;
        double N = Constants::EARTH_SEMIMAJOR / std::sqrt(1 - e2 * sinLat * sinLat);

        Vector3 ecef;
        ecef[0] = (N + h) * cosLat * cosLon;
        ecef[1] = (N + h) * cosLat * sinLon;
        ecef[2] = (N * (1 - Constants::EARTH_FLATTENING) * (1 - Constants::EARTH_FLATTENING) + h) * sinLat;

        return ecef;
    }

    Coordinate::Vector3 Coordinate::EcefToGps(const Vector3& ecef) {
        double x = ecef[0];
        double y = ecef[1];
        double z = ecef[2];

        double p = std::sqrt(x * x + y * y);

        // 经度
        double lon = std::atan2(y, x) * Constants::RAD_TO_DEG;

        // 纬度（迭代计算）
        double e2 = Constants::ECCENTRICITY_FIRST * Constants::ECCENTRICITY_FIRST;
        double lat = std::atan2(z, p * (1 - Constants::EARTH_FLATTENING));
        double N, h;

        for (int iter = 0; iter < 10; iter++) {
            double sinLat = std::sin(lat * Constants::DEG_TO_RAD);
            double cosLat = std::cos(lat * Constants::DEG_TO_RAD);
            N = Constants::EARTH_SEMIMAJOR / std::sqrt(1 - e2 * sinLat * sinLat);
            h = p / cosLat - N;

            double lat_new = std::atan2(z, p * (1 - e2 * N / (N + h)));
            if (std::abs(lat_new - lat) < 1e-12) {
                lat = lat_new;
                break;
            }
            lat = lat_new;
        }
        lat *= Constants::RAD_TO_DEG;

        return Vector3(lon, lat, h);
    }

    Coordinate::Vector3 Coordinate::EcefToGpsNewton(const Vector3& ecef) {
        // 使用牛顿迭代法求解，精度更高
        double x = ecef[0];
        double y = ecef[1];
        double z = ecef[2];

        double p = std::sqrt(x * x + y * y);
        double lon = std::atan2(y, x);

        // 初始估计
        double lat = std::atan2(z, p * 0.99);  // 略微压扁
        double h = 0.0;

        double a = Constants::EARTH_SEMIMAJOR;
        double f = Constants::EARTH_FLATTENING;
        double e2 = 2 * f - f * f;

        // 牛顿迭代
        for (int iter = 0; iter < 20; iter++) {
            double sinLat = std::sin(lat);
            double cosLat = std::cos(lat);
            double N = a / std::sqrt(1 - e2 * sinLat * sinLat);

            // 计算残差
            double p_calc = (N + h) * cosLat;
            double z_calc = (N * (1 - e2) + h) * sinLat;

            double dp = p - p_calc;
            double dz = z - z_calc;

            // 雅可比矩阵
            double dN_dlat = a * e2 * sinLat * cosLat / std::pow(1 - e2 * sinLat * sinLat, 1.5);
            double dp_dlat = dN_dlat * cosLat - (N + h) * sinLat;
            double dp_dh = cosLat;
            double dz_dlat = dN_dlat * (1 - e2) * sinLat + (N * (1 - e2) + h) * cosLat;
            double dz_dh = sinLat;

            // 求解线性方程组
            double det = dp_dlat * dz_dh - dp_dh * dz_dlat;
            if (std::abs(det) < 1e-20) break;

            double dlat = (dz_dh * dp - dp_dh * dz) / det;
            double dh = (-dz_dlat * dp + dp_dlat * dz) / det;

            lat += dlat;
            h += dh;

            // 检查收敛
            if (std::abs(dlat) < 1e-15 && std::abs(dh) < 1e-8) {
                break;
            }
        }

        return Vector3(lon * Constants::RAD_TO_DEG, lat * Constants::RAD_TO_DEG, h);
    }

    // ============================================================
    // NED 坐标系
    // ============================================================

    Coordinate::Matrix3 Coordinate::EcefToNedMatrix(double longitude, double latitude) {
        double lon = longitude * Constants::DEG_TO_RAD;
        double lat = latitude * Constants::DEG_TO_RAD;

        Matrix3 R_y = RotationMatrix(Constants::PI / 2.0 - lon, 1);  // 绕Y轴
        Matrix3 R_x = RotationMatrix(Constants::PI / 2.0 - lat, 2);  // 绕X轴
        Matrix3 R_z = RotationMatrix(lat, 3);                        // 绕Z轴

        return (R_z * R_x * R_y).transpose();
    }

    Coordinate::Matrix3 Coordinate::NedToEcefMatrix(double longitude, double latitude) {
        return EcefToNedMatrix(longitude, latitude).transpose();
    }

    Coordinate::Vector3 Coordinate::EcefToNed(const Vector3& ecef_pos, const Vector3& ref_gps) {
        Vector3 ref_ecef = GpsToEcef(ref_gps);
        Matrix3 E2N = EcefToNedMatrix(ref_gps[0], ref_gps[1]);
        return E2N * (ecef_pos - ref_ecef);
    }

    Coordinate::Vector3 Coordinate::NedToEcef(const Vector3& ned_pos, const Vector3& ref_gps) {
        Vector3 ref_ecef = GpsToEcef(ref_gps);
        Matrix3 N2E = NedToEcefMatrix(ref_gps[0], ref_gps[1]);
        return ref_ecef + N2E * ned_pos;
    }

    // ============================================================
    // 速度转换
    // ============================================================

    void Coordinate::VelocityToNed(double theta, double phi_v, double V, Vector3& vn) {
        vn[0] = V * std::cos(theta) * std::cos(phi_v);
        vn[1] = V * std::sin(theta);
        vn[2] = -V * std::cos(theta) * std::sin(phi_v);
    }

    void Coordinate::NedToEcefVelocity(const Vector3& ned_vel, const Vector3& gps_pos, Vector3& ecef_vel) {
        Matrix3 N2E = NedToEcefMatrix(gps_pos[0], gps_pos[1]);
        ecef_vel = N2E * ned_vel;
    }

    // ============================================================
    // 旋转矩阵
    // ============================================================

    Coordinate::Matrix3 Coordinate::RotationMatrix(double angle, int axis) {
        Matrix3 R = Matrix3::Identity();
        double c = std::cos(angle);
        double s = std::sin(angle);

        switch (axis) {
            case 1:  // X轴
                R(1, 1) = c;  R(1, 2) = s;
                R(2, 1) = -s; R(2, 2) = c;
                break;
            case 2:  // Y轴
                R(0, 0) = c;  R(0, 2) = -s;
                R(2, 0) = s;  R(2, 2) = c;
                break;
            case 3:  // Z轴
                R(0, 0) = c;  R(0, 1) = s;
                R(1, 0) = -s; R(1, 1) = c;
                break;
            default:
                // 保持单位矩阵
                break;
        }

        return R;
    }

} // namespace SimTools
