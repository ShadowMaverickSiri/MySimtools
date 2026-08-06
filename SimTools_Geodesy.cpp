// ============================================================
// SimTools v2.0 - 地理计算模块实现
// ============================================================

#include "SimTools_v2.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace SimTools {

    // ============================================================
    // 距离计算
    // ============================================================

    double Geodesy::GreatCircleDistance(double lon1, double lat1,
                                       double lon2, double lat2) {
        double lon1_rad = lon1 * Constants::DEG_TO_RAD;
        double lat1_rad = lat1 * Constants::DEG_TO_RAD;
        double lon2_rad = lon2 * Constants::DEG_TO_RAD;
        double lat2_rad = lat2 * Constants::DEG_TO_RAD;

        double dlon = lon2_rad - lon1_rad;
        double dlat = lat2_rad - lat1_rad;

        double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                  std::cos(lat1_rad) * std::cos(lat2_rad) *
                  std::sin(dlon / 2) * std::sin(dlon / 2);

        a = std::max(0.0, std::min(1.0, a));
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

        return Constants::EARTH_SEMIMAJOR * c;
    }

    double Geodesy::HaversineDistance(double lon1, double lat1,
                                     double lon2, double lat2) {
        // Haversine 公式与 GreatCircle 类似，只是表达方式不同
        return GreatCircleDistance(lon1, lat1, lon2, lat2);
    }

    double Geodesy::VincentyDistance(double lon1, double lat1,
                                    double lon2, double lat2) {
        double distance, az1, az2;
        VincentyInverse(lon1, lat1, lon2, lat2, distance, az1, az2);
        return distance;
    }

    // ============================================================
    // 方位角计算
    // ============================================================

    double Geodesy::Azimuth(const Vector3d& gps1,
                           const Vector3d& gps2) {
        double azimuth, distance;
        AzimuthAndDistance(gps1, gps2, azimuth, distance);
        return azimuth;
    }

    void Geodesy::AzimuthAndDistance(const Vector3d& gps1,
                                    const Vector3d& gps2,
                                    double& azimuth,
                                    double& distance) {
        double lon1 = gps1[0] * Constants::DEG_TO_RAD;
        double lat1 = gps1[1] * Constants::DEG_TO_RAD;
        double lon2 = gps2[0] * Constants::DEG_TO_RAD;
        double lat2 = gps2[1] * Constants::DEG_TO_RAD;

        double dlon = lon2 - lon1;

        double y = std::sin(dlon) * std::cos(lat2);
        double x = std::cos(lat1) * std::sin(lat2) -
                  std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

        azimuth = std::atan2(y, x) * Constants::RAD_TO_DEG;
        azimuth = Math::Regulate360(azimuth);

        distance = GreatCircleDistance(gps1[0], gps1[1], gps2[0], gps2[1]);
    }

    // ============================================================
    // Vincenty 公式（高精度椭球面计算）
    // ============================================================

    void Geodesy::VincentyInverse(double lon1, double lat1, double lon2, double lat2,
                                 double& distance, double& azimuth1, double& azimuth2) {
        // 将角度转换为弧度
        double L1 = lon1 * Constants::DEG_TO_RAD;
        double B1 = lat1 * Constants::DEG_TO_RAD;
        double L2 = lon2 * Constants::DEG_TO_RAD;
        double B2 = lat2 * Constants::DEG_TO_RAD;

        double a = Constants::EARTH_SEMIMAJOR;
        double f = Constants::EARTH_FLATTENING;
        double b = a * (1 - f);

        // 计算归化纬度
        double tanU1 = (1 - f) * std::tan(B1);
        double tanU2 = (1 - f) * std::tan(B2);

        double cosU1 = 1.0 / std::sqrt(1 + tanU1 * tanU1);
        double cosU2 = 1.0 / std::sqrt(1 + tanU2 * tanU2);
        double sinU1 = tanU1 * cosU1;
        double sinU2 = tanU2 * cosU2;

        double L = Math::RegulatePi(L2 - L1);
        double lambda = L;
        double lambdaPrev = lambda;
        int iterLimit = 100;
        double cosSqAlpha = 0.0;
        double sinSigma = 0.0;
        double cosSigma = 1.0;
        double sigma = 0.0;
        double cos2SigmaM = 0.0;
        double sinAlpha = 0.0;
        bool converged = false;

        // 迭代求解 lambda
        for (int iter = 0; iter < iterLimit; iter++) {
            double sinLambda = std::sin(lambda);
            double cosLambda = std::cos(lambda);

            double sinSigmaSq = cosU2 * sinLambda * cosU2 * sinLambda +
                               (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda) *
                               (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda);

            sinSigma = std::sqrt(sinSigmaSq);
            if (sinSigma == 0) {
                // 共点
                distance = 0;
                azimuth1 = 0;
                azimuth2 = 0;
                return;
            }

            cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
            sigma = std::atan2(sinSigma, cosSigma);
            sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
            sinAlpha = std::max(-1.0, std::min(1.0, sinAlpha));
            cosSqAlpha = std::max(0.0, 1 - sinAlpha * sinAlpha);
            if (cosSqAlpha < 1e-15) {
                // 赤道大地线极限
                cos2SigmaM = 0.0;
            } else {
                cos2SigmaM = cosSigma - 2 * sinU1 * sinU2 / cosSqAlpha;
            }

            double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
            lambdaPrev = lambda;
            lambda = L + (1 - C) * f * sinAlpha *
                     (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma *
                      (-1 + 2 * cos2SigmaM * cos2SigmaM)));

            if (std::abs(lambda - lambdaPrev) < 1e-12) {
                converged = true;
                break;
            }
        }

        if (!converged) {
            throw std::runtime_error("VincentyInverse failed to converge for the supplied points");
        }

        // 计算距离
        double u2 = cosSqAlpha * (a * a - b * b) / (b * b);
        double A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)));
        double B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)));

        double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 *
                           (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) -
                            B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) *
                            (-3 + 4 * cos2SigmaM * cos2SigmaM)));

        distance = b * A * (sigma - deltaSigma);

        // 计算方位角
        azimuth1 = std::atan2(cosU2 * std::sin(lambda),
                             cosU1 * sinU2 - sinU1 * cosU2 * std::cos(lambda)) * Constants::RAD_TO_DEG;
        azimuth2 = std::atan2(cosU1 * std::sin(lambda),
                             -sinU1 * cosU2 + cosU1 * sinU2 * std::cos(lambda)) * Constants::RAD_TO_DEG;

        azimuth1 = Math::Regulate360(azimuth1);
        azimuth2 = Math::Regulate360(azimuth2);
    }

    void Geodesy::VincentyDirect(double lon1, double lat1,
                               double azimuth, double distance,
                               double& lon2, double& lat2, double& azimuth2) {
        double L1 = lon1 * Constants::DEG_TO_RAD;
        double B1 = lat1 * Constants::DEG_TO_RAD;
        double alpha1 = azimuth * Constants::DEG_TO_RAD;
        double s = distance;

        double a = Constants::EARTH_SEMIMAJOR;
        double f = Constants::EARTH_FLATTENING;
        double b = a * (1 - f);

        // 计算归化纬度
        double tanU1 = (1 - f) * std::tan(B1);
        double cosU1 = 1.0 / std::sqrt(1 + tanU1 * tanU1);
        double sinU1 = tanU1 * cosU1;

        double alpha1_cos = std::cos(alpha1);
        double alpha1_sin = std::sin(alpha1);

        double sigma1 = std::atan2(tanU1, alpha1_cos);
        double sinAlpha = cosU1 * alpha1_sin;
        double cosSqAlpha = 1 - sinAlpha * sinAlpha;

        double u2 = cosSqAlpha * (a * a - b * b) / (b * b);
        double A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)));
        double B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)));

        double sigma = s / (b * A);
        double sigmaPrev = sigma;
        int iterLimit = 100;
        bool converged = false;

        // 迭代求解 sigma
        for (int iter = 0; iter < iterLimit; iter++) {
            double cos2SigmaM = std::cos(2 * sigma1 + sigma);
            double sinSigma = std::sin(sigma);
            double cosSigma = std::cos(sigma);

            double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 *
                               (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) -
                                B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) *
                                (-3 + 4 * cos2SigmaM * cos2SigmaM)));

            sigmaPrev = sigma;
            sigma = s / (b * A) + deltaSigma;

            if (std::abs(sigma - sigmaPrev) < 1e-12) {
                converged = true;
                break;
            }
        }

        if (!converged) {
            throw std::runtime_error("VincentyDirect failed to converge for the supplied distance");
        }

        double cos2SigmaM = std::cos(2 * sigma1 + sigma);
        double sinSigma = std::sin(sigma);
        double cosSigma = std::cos(sigma);

        // 纬度分母项，与经度公式分母不同
        double B2_denom = sinU1 * sinSigma - cosU1 * cosSigma * alpha1_cos;
        double B2 = atan2(sinU1 * cosSigma + cosU1 * sinSigma * alpha1_cos,
                          (1 - f) * std::sqrt(sinAlpha * sinAlpha + B2_denom * B2_denom));

        double lambda = std::atan2(sinSigma * alpha1_sin,
                                  cosU1 * cosSigma - sinU1 * sinSigma * alpha1_cos);

        double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
        double L = lambda - (1 - C) * f * sinAlpha *
                  (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma *
                   (-1 + 2 * cos2SigmaM * cos2SigmaM)));

        double L2 = L1 + L;

        // 终点处沿原大地线继续前进的前向方位角
        double alpha2 = std::atan2(sinAlpha, -sinU1 * sinSigma + cosU1 * cosSigma * alpha1_cos);

        lon2 = Math::Regulate180(L2 * Constants::RAD_TO_DEG);
        lat2 = B2 * Constants::RAD_TO_DEG;
        azimuth2 = alpha2 * Constants::RAD_TO_DEG;
        azimuth2 = Math::Regulate360(azimuth2);
    }

    // ============================================================
    // 站心计算
    // ============================================================

    Vector3d Geodesy::TargetFromSite(const Vector3d& site_gps,
                                          double azimuth,
                                          double target_height,
                                          double slant_range) {
        if (slant_range < 0.0) {
            throw std::invalid_argument("slant_range must be non-negative");
        }

        Vector3d site_ecef = Coordinate::GpsToEcef(site_gps);
        Matrix3d E2N = Coordinate::EcefToNedMatrix(site_gps[0], site_gps[1]);
        double az_rad = azimuth * Constants::DEG_TO_RAD;

        if (slant_range == 0.0) {
            if (std::abs(target_height - site_gps[2]) > 1e-9) {
                throw std::invalid_argument("zero slant_range requires the target to equal the site height");
            }
            return site_gps;
        }

        // 二分求解满足目标椭球高的仰角
        auto targetAtElevation = [&](double elevation) {
            Vector3d target_ned;
            double horizontal = slant_range * std::cos(elevation);
            target_ned[0] = horizontal * std::cos(az_rad);
            target_ned[1] = horizontal * std::sin(az_rad);
            target_ned[2] = -slant_range * std::sin(elevation);
            return Coordinate::EcefToGps(site_ecef + E2N.transpose() * target_ned);
        };

        double low = -Constants::PI / 2;
        double high = Constants::PI / 2;
        double fLow = targetAtElevation(low)[2] - target_height;
        double fHigh = targetAtElevation(high)[2] - target_height;
        if (fLow * fHigh > 0.0) {
            throw std::invalid_argument("target height is unreachable at the supplied slant_range");
        }

        Vector3d target_gps = site_gps;
        for (int iter = 0; iter < 80; ++iter) {
            double mid = (low + high) / 2;
            target_gps = targetAtElevation(mid);
            double fMid = target_gps[2] - target_height;
            if (std::abs(fMid) < 1e-7) {
                break;
            }
            if (fLow * fMid <= 0.0) {
                high = mid;
            } else {
                low = mid;
                fLow = fMid;
            }
        }
        target_gps[2] = target_height;
        return target_gps;
    }

    double Geodesy::SiteAzimuth(const Vector3d& gpsA, const Vector3d& gpsB) {
        // 计算站心坐标系下的方位角
        Vector3d posA = Coordinate::GpsToEcef(gpsA);
        Vector3d posB = Coordinate::GpsToEcef(gpsB);

        Matrix3d E2N = Coordinate::EcefToNedMatrix(gpsA[0], gpsA[1]);
        Vector3d rel_pos = E2N * (posB - posA);

        double azimuth = std::atan2(rel_pos[1], rel_pos[0]) * Constants::RAD_TO_DEG;
        return Math::Regulate360(azimuth);
    }

    double Geodesy::SiteDistance(const Vector3d& gpsA, const Vector3d& gpsB) {
        Vector3d posA = Coordinate::GpsToEcef(gpsA);
        Vector3d posB = Coordinate::GpsToEcef(gpsB);
        return (posB - posA).norm();
    }

} // namespace SimTools
