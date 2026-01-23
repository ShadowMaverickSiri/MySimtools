// ============================================================
// SimTools v2.0 - 时间工具模块实现
// ============================================================

#include "SimTools_v2.h"
#include <chrono>
#include <ctime>
#include <cmath>

namespace SimTools {

    // ============================================================
    // Unix 时间戳
    // ============================================================

    double Time::GetUnixTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration<double>(duration).count();
    }

    // ============================================================
    // GPS 时间转换
    // ============================================================

    Time::GpsTime Time::UnixToGpsTime(double unix_time) {
        // GPS 时间始于 1980-01-06 00:00:00 UTC
        // Unix 时间始于 1970-01-01 00:00:00 UTC
        // 相差 315964800 秒（从 1970-01-01 到 1980-01-06）

        constexpr double GPS_TO_UNIX_OFFSET = 315964800.0;
        constexpr double SECONDS_PER_WEEK = 7 * 24 * 3600.0;

        double gps_seconds = unix_time - GPS_TO_UNIX_OFFSET;
        int week = static_cast<int>(gps_seconds / SECONDS_PER_WEEK);
        double seconds = gps_seconds - week * SECONDS_PER_WEEK;

        return {week, seconds};
    }

    double Time::GpsTimeToUnix(const GpsTime& gps_time) {
        constexpr double GPS_TO_UNIX_OFFSET = 315964800.0;
        constexpr double SECONDS_PER_WEEK = 7 * 24 * 3600.0;

        return GPS_TO_UNIX_OFFSET + gps_time.week * SECONDS_PER_WEEK + gps_time.seconds;
    }

    // ============================================================
    // UTC 到 GPS 时间秒
    // ============================================================

    double Time::UtcToGpsSeconds(int year, int month, int day,
                                int hour, int minute, double second) {
        // 计算从 1980-01-06 到给定日期的天数
        // 使用简化的儒略日计算

        // 如果日期在 1980 年之前，返回负值
        if (year < 1980) {
            return -1.0;
        }

        // 简化计算：只处理 1980 年之后的日期
        // 实际应用中应使用更精确的算法（考虑闰秒等）

        // 估算从 1980-01-06 开始的秒数
        int days_since_1980 = 0;

        // 计算整年
        for (int y = 1980; y < year; ++y) {
            days_since_1980 += 365;
            if ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0)) {
                days_since_1980 += 1;  // 闰年
            }
        }

        // 每月天数表（非闰年）
        const int days_per_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

        // 计算整月
        for (int m = 1; m < month; ++m) {
            days_since_1980 += days_per_month[m - 1];
            if (m == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
                days_since_1980 += 1;  // 闰年二月
            }
        }

        // 加上当月天数
        days_since_1980 += day - 6;  // 从 1月6日开始

        // 转换为秒
        double gps_seconds = days_since_1980 * 86400.0 +
                           hour * 3600.0 +
                           minute * 60.0 +
                           second;

        return gps_seconds;
    }

    // ============================================================
    // 格林威治恒星时
    // ============================================================

    double Time::GreenwhichSiderealTime(double mjd) {
        // 计算格林威治平恒星时（GMST）
        // mjd: 简化儒略日（Modified Julian Date）

        // 从 J2000.0 开始的儒略世纪数
        double jd = mjd + 2400000.5;
        double d = jd - 2451545.0;
        double T = d / 36525.0;

        // GMST 角度（单位：度）
        double gmst_deg = 280.46061837 + 360.98564736629 * d +
                         0.000387933 * T * T - T * T * T / 38710000.0;

        // 规范化到 [0, 360) 度
        gmst_deg = gmst_deg - 360.0 * std::floor(gmst_deg / 360.0);

        // 转换为弧度
        return gmst_deg * Constants::DEG_TO_RAD;
    }

} // namespace SimTools
