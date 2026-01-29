// ============================================================
// SimTools v2.0 - 实现示例
// 这是一个演示文件，展示如何实现重构后的 SimTools
// ============================================================

#include "SimTools_v2.h"
#include <chrono>
#include <ctime>
#include <iomanip>

using namespace SimTools;

// ============================================================
// 插值算法实现
// ============================================================

namespace SimTools {

    double Interpolation::Linear(double x,
                                 const std::vector<double>& xx,
                                 const std::vector<double>& yy) {
        // 输入验证
        if (xx.empty() || xx.size() != yy.size()) {
            return 0.0;
        }

        if (xx.size() == 1) {
            return yy[0];
        }

        // 查找插值区间
        size_t idx = FindIndex(x, xx);

        if (idx == 0) {
            return yy[0];  // 外推：使用边界值
        }
        if (idx >= xx.size()) {
            return yy.back();
        }

        // 线性插值
        double t = (x - xx[idx - 1]) / (xx[idx] - xx[idx - 1]);
        return yy[idx - 1] + t * (yy[idx] - yy[idx - 1]);
    }

    double Interpolation::Lagrange7(double x,
                                    const std::vector<double>& xx,
                                    const std::vector<double>& yy) {
        int n = static_cast<int>(xx.size());

        if (n < 1 || xx.size() != yy.size()) {
            return 0.0;
        }

        if (n <= 2) {
            return Linear(x, xx, yy);
        }

        // 找到 x 的位置
        int i = 0;
        while (i < n && xx[i] < x) i++;

        // 取前后各 3-4 个点（共 7 点或更少）
        int k = i - 4;
        if (k < 0) k = 0;
        int m = i + 3;
        if (m > n - 1) m = n - 1;

        // 拉格朗日插值
        double result = 0.0;
        for (int idx = k; idx <= m; idx++) {
            double term = yy[idx];
            for (int j = k; j <= m; j++) {
                if (j != idx) {
                    term *= (x - xx[j]) / (xx[idx] - xx[j]);
                }
            }
            result += term;
        }

        return result;
    }

    double Interpolation::LagrangeGlobal(double x,
                                         const std::vector<double>& xx,
                                         const std::vector<double>& yy) {
        int n = static_cast<int>(xx.size());

        if (n < 1 || xx.size() != yy.size()) {
            return 0.0;
        }

        if (n == 1) {
            return yy[0];
        }

        // 全局拉格朗日插值
        double result = 0.0;
        for (int i = 0; i < n; i++) {
            double term = yy[i];
            for (int j = 0; j < n; j++) {
                if (j != i) {
                    term *= (x - xx[j]) / (xx[i] - xx[j]);
                }
            }
            result += term;
        }

        return result;
    }

    int Interpolation::FindIndex(double x, const std::vector<double>& xx) {
        if (xx.empty()) {
            return 0;
        }

        // 判断是升序还是降序
        bool ascending = (xx.size() > 1) && (xx[1] > xx[0]);

        if (ascending) {
            auto it = std::upper_bound(xx.begin(), xx.end(), x);
            return static_cast<int>(std::distance(xx.begin(), it));
        } else {
            auto it = std::lower_bound(xx.begin(), xx.end(), x, std::greater<double>());
            return static_cast<int>(std::distance(xx.begin(), it));
        }
    }

    std::pair<int, int> Interpolation::FindIndex2D(double u, double v,
                                                    const std::vector<double>& x,
                                                    const std::vector<double>& y) {
        return {FindIndex(u, x), FindIndex(v, y)};
    }

} // namespace SimTools


// ============================================================
// 坐标转换实现
// ============================================================

namespace SimTools {

    Coordinate::Vector3 Coordinate::GpsToEcef(const Vector3& gps) {
        double lon = gps[0] * Constants::DEG_TO_RAD;
        double lat = gps[1] * Constants::DEG_TO_RAD;
        double h = gps[2];

        double sinLat = std::sin(lat);
        double cosLat = std::cos(lat);
        double sinLon = std::sin(lon);
        double cosLon = std::cos(lon);

        // 卯酉圈曲率半径
        double N = Constants::EARTH_SEMIMAJOR /
                  std::sqrt(1 - Constants::ECCENTRICITY_FIRST * Constants::ECCENTRICITY_FIRST *
                           sinLat * sinLat);

        Vector3 ecef;
        ecef[0] = (N + h) * cosLat * cosLon;
        ecef[1] = (N + h) * cosLat * sinLon;
        ecef[2] = (N * (1 - Constants::EARTH_FLATTENING) *
                  (1 - Constants::EARTH_FLATTENING) + h) * sinLat;

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
        double lat = std::atan2(z, p * (1 - Constants::EARTH_FLATTENING));
        double N, h;

        for (int iter = 0; iter < 5; iter++) {
            double sinLat = std::sin(lat * Constants::DEG_TO_RAD);
            N = Constants::EARTH_SEMIMAJOR /
                std::sqrt(1 - Constants::ECCENTRICITY_FIRST * Constants::ECCENTRICITY_FIRST *
                         sinLat * sinLat);
            h = p / std::cos(lat * Constants::DEG_TO_RAD) - N;
            lat = std::atan2(z, p * (1 - Constants::ECCENTRICITY_FIRST * Constants::ECCENTRICITY_FIRST *
                                    N / (N + h)));
        }
        lat *= Constants::RAD_TO_DEG;

        return Vector3(lon, lat, h);
    }

    Coordinate::Vector3 Coordinate::EcefToGpsNewton(const Vector3& ecef) {
        // 牛顿法求解（精度更高）
        // 初始估计
        Vector3 gps = EcefToGps(ecef);

        // TODO: 实现完整的牛顿迭代
        // 这里简化返回，实际应用中应该迭代到收敛

        return gps;
    }

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

    void Coordinate::VelocityToNed(double theta, double phi_v, double V, Vector3& vn) {
        vn[0] = V * std::cos(theta) * std::cos(phi_v);
        vn[1] = V * std::sin(theta);
        vn[2] = -V * std::cos(theta) * std::sin(phi_v);
    }

    void Coordinate::NedToEcefVelocity(const Vector3& ned_vel, const Vector3& gps_pos, Vector3& ecef_vel) {
        Matrix3 N2E = NedToEcefMatrix(gps_pos[0], gps_pos[1]);
        ecef_vel = N2E * ned_vel;
    }

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


// ============================================================
// 地理计算实现
// ============================================================

namespace SimTools {

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

    double Geodesy::Azimuth(const Vector3d& gps1, const Vector3d& gps2) {
        double azimuth, distance;
        AzimuthAndDistance(gps1, gps2, azimuth, distance);
        return azimuth;
    }

    void Geodesy::AzimuthAndDistance(const Vector3d& gps1, const Vector3d& gps2,
                                    double& azimuth, double& distance) {
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

    void Geodesy::VincentyInverse(double lon1, double lat1, double lon2, double lat2,
                                 double& distance, double& azimuth1, double& azimuth2) {
        // 简化版本，使用 Haversine
        // 完整的 Vincenty 反解需要迭代求解
        double az, dist;
        AzimuthAndDistance(Vector3d(lon1, lat1, 0), Vector3d(lon2, lat2, 0), az, dist);
        distance = dist;
        azimuth1 = az;
        azimuth2 = Math::Regulate180(az + 180.0);
    }

    void Geodesy::VincentyDirect(double lon1, double lat1,
                               double azimuth, double distance,
                               double& lon2, double& lat2, double& azimuth2) {
        // 简化版本：使用大圆航法
        double lon1_rad = lon1 * Constants::DEG_TO_RAD;
        double lat1_rad = lat1 * Constants::DEG_TO_RAD;
        double az_rad = azimuth * Constants::DEG_TO_RAD;

        double angular_distance = distance / Constants::EARTH_SEMIMAJOR;

        double lat2_rad = std::asin(std::sin(lat1_rad) * std::cos(angular_distance) +
                                    std::cos(lat1_rad) * std::sin(angular_distance) * std::cos(az_rad));

        double lon2_rad = lon1_rad + std::atan2(
            std::sin(az_rad) * std::sin(angular_distance) * std::cos(lat1_rad),
            std::cos(angular_distance) - std::sin(lat1_rad) * std::sin(lat2_rad)
        );

        lat2 = lat2_rad * Constants::RAD_TO_DEG;
        lon2 = lon2_rad * Constants::RAD_TO_DEG;
        azimuth2 = Math::Regulate360(azimuth + 180.0);
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
        return (posB - posA).norm();  // 使用成员函数避免重载歧义
    }

} // namespace SimTools


// ============================================================
// 大气参数实现
// ============================================================

namespace SimTools {

    Atmosphere::Parameters Atmosphere::GetParameters(double height_meters) {
        Parameters params;

        // 限制高度范围
        double h = std::max(0.0, std::min(height_meters, 90000.0));

        // 简化大气模型（实际应用中应使用 1976 标准大气模型）
        if (h <= 11000) {
            // 对流层
            params.temperature = 288.15 - 0.0065 * h;
            params.pressure = 101325.0 * std::pow(1 - 0.0065 * h / 288.15, 5.25588);
        } else if (h <= 20000) {
            // 平流层下层
            params.temperature = 216.65;
            params.pressure = 22632.0 * std::exp(-(h - 11000) / 6341.62);
        } else if (h <= 32000) {
            // 平流层中层
            params.temperature = 216.65 + 0.001 * (h - 20000);
            params.pressure = 5474.87 * std::pow(1 + 0.001 * (h - 20000) / 216.65, -34.163);
        } else if (h <= 47000) {
            // 平流层上层
            params.temperature = 228.65 + 0.0028 * (h - 32000);
            params.pressure = 868.015 * std::pow(1 + 0.0028 * (h - 32000) / 228.65, -12.201);
        } else if (h <= 51000) {
            // 中间层下层
            params.temperature = 270.65;
            params.pressure = 110.906 * std::exp(-(h - 47000) / 7922.0);
        } else {
            // 中间层上层
            params.temperature = 270.65 - 0.0028 * (h - 51000);
            params.pressure = 75.944 * std::pow(1 - 0.0028 * (h - 51000) / 270.65, 12.201);
        }

        params.gravity = Constants::GRAVITY_SEA_LEVEL *
                        std::pow(Constants::EARTH_SEMIMAJOR / (Constants::EARTH_SEMIMAJOR + h), 2);
        params.density = params.pressure / (Constants::GAS_CONSTANT_AIR * params.temperature);
        params.sound_speed = std::sqrt(1.4 * Constants::GAS_CONSTANT_AIR * params.temperature);

        return params;
    }

    double Atmosphere::Gravity(double height_meters) {
        return GetParameters(height_meters).gravity;
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

    double Atmosphere::VelocityFromMach(double mach, double height_meters) {
        return mach * SoundSpeed(height_meters);
    }

    double Atmosphere::MachFromVelocity(double velocity, double height_meters) {
        return velocity / SoundSpeed(height_meters);
    }

    double Atmosphere::DynamicPressure(double velocity, double height_meters) {
        return 0.5 * Density(height_meters) * velocity * velocity;
    }

} // namespace SimTools


// ============================================================
// 随机数生成实现
// ============================================================

namespace SimTools {

    void Random::Seed(unsigned int seed) {
        if (seed == 0) {
            // 使用当前时间作为种子
            seed = static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count());
        }
        std::srand(seed);
    }

    double Random::Uniform01() {
        return static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) + 1.0);
    }

    double Random::Uniform(double a, double b) {
        return a + (b - a) * Uniform01();
    }

    int Random::UniformInt(int a, int b) {
        return a + std::rand() % (b - a + 1);
    }

    double Random::Normal01() {
        // Box-Muller 变换
        double u1 = Uniform01();
        double u2 = Uniform01();
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * Constants::PI * u2);
    }

    double Random::Normal(double mu, double sigma) {
        return mu + sigma * Normal01();
    }

    double Random::Exponential(double lambda) {
        return -std::log(1.0 - Uniform01()) / lambda;
    }

    double Random::Weibull(double shape, double scale) {
        return scale * std::pow(-std::log(1.0 - Uniform01()), 1.0 / shape);
    }

} // namespace SimTools


// ============================================================
// 文件 I/O 实现
// ============================================================

namespace SimTools {

    MatrixXd FileIO::ReadMatrix(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + filename);
        }

        std::vector<std::vector<double>> data;
        std::string line;

        while (std::getline(file, line)) {
            // 跳过空行和注释行
            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::vector<double> row;
            std::stringstream ss(line);
            double value;

            while (ss >> value) {
                row.push_back(value);
            }

            if (!row.empty()) {
                data.push_back(row);
            }
        }

        file.close();

        if (data.empty()) {
            return MatrixXd(0, 0);
        }

        // 转换为 Eigen 矩阵
        int rows = static_cast<int>(data.size());
        int cols = static_cast<int>(data[0].size());

        MatrixXd matrix(rows, cols);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                matrix(i, j) = data[i][j];
            }
        }

        return matrix;
    }

    std::vector<std::vector<double>> FileIO::ReadTable(const std::string& filename) {
        MatrixXd matrix = ReadMatrix(filename);

        std::vector<std::vector<double>> table;
        table.reserve(matrix.rows());

        for (int i = 0; i < matrix.rows(); ++i) {
            std::vector<double> row;
            row.reserve(matrix.cols());
            for (int j = 0; j < matrix.cols(); ++j) {
                row.push_back(matrix(i, j));
            }
            table.push_back(row);
        }

        return table;
    }

    std::vector<double> FileIO::ReadColumn(const std::string& filename, int column_index) {
        MatrixXd matrix = ReadMatrix(filename);

        if (matrix.cols() <= column_index) {
            throw std::runtime_error("Column index out of range");
        }

        std::vector<double> column;
        column.reserve(matrix.rows());

        for (int i = 0; i < matrix.rows(); ++i) {
            column.push_back(matrix(i, column_index));
        }

        return column;
    }

    bool FileIO::WriteMatrix(const std::string& filename,
                            const MatrixXd& matrix,
                            bool scientific,
                            int precision) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }

        if (scientific) {
            file << std::scientific << std::setprecision(precision);
        } else {
            file << std::fixed << std::setprecision(precision);
        }

        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                file << matrix(i, j);
                if (j < matrix.cols() - 1) {
                    file << " ";
                }
            }
            file << "\n";
        }

        file.close();
        return true;
    }

    bool FileIO::WriteVector(const std::string& filename,
                            const std::vector<double>& data,
                            bool scientific,
                            int precision) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }

        if (scientific) {
            file << std::scientific << std::setprecision(precision);
        } else {
            file << std::fixed << std::setprecision(precision);
        }

        for (size_t i = 0; i < data.size(); ++i) {
            file << data[i] << "\n";
        }

        file.close();
        return true;
    }

    int FileIO::CountLines(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return 0;
        }

        int count = 0;
        std::string line;
        while (std::getline(file, line)) {
            if (!line.empty() && line[0] != '#') {
                count++;
            }
        }

        file.close();
        return count;
    }

    int FileIO::CountColumns(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return 0;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (!line.empty() && line[0] != '#') {
                std::stringstream ss(line);
                int count = 0;
                double value;
                while (ss >> value) {
                    count++;
                }
                file.close();
                return count;
            }
        }

        file.close();
        return 0;
    }

    std::string FileIO::ToString(int value) {
        return std::to_string(value);
    }

    std::string FileIO::ToString(double value, int precision) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(precision) << value;
        return ss.str();
    }

    bool FileIO::Exists(const std::string& filename) {
        std::ifstream file(filename);
        return file.good();
    }

} // namespace SimTools


// ============================================================
// 数值计算实现
// ============================================================

namespace SimTools {

    VectorXd Numerical::RungeKutta4(const OdeFunction& f,
                                          double t0,
                                          const VectorXd& y0,
                                          double h,
                                          int steps) {
        int n = static_cast<int>(y0.size());
        VectorXd y = y0;
        double t = t0;

        for (int step = 0; step < steps; ++step) {
            VectorXd k1 = f(t, y);
            VectorXd k2 = f(t + 0.5 * h, y + 0.5 * h * k1);
            VectorXd k3 = f(t + 0.5 * h, y + 0.5 * h * k2);
            VectorXd k4 = f(t + h, y + h * k3);

            y = y + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
            t += h;
        }

        return y;
    }

    double Numerical::Bisection(const std::function<double(double)>& f,
                               double a, double b,
                               double tol) {
        double fa = f(a);
        double fb = f(b);

        if (fa * fb > 0) {
            throw std::runtime_error("Function has same sign at endpoints");
        }

        double c;
        while ((b - a) > tol) {
            c = (a + b) / 2.0;
            double fc = f(c);

            if (std::abs(fc) < tol) {
                return c;
            }

            if (fa * fc < 0) {
                b = c;
                fb = fc;
            } else {
                a = c;
                fa = fc;
            }
        }

        return (a + b) / 2.0;
    }

    double Numerical::Newton(const std::function<double(double)>& f,
                            const std::function<double(double)>& df,
                            double x0,
                            double tol,
                            int max_iter) {
        double x = x0;

        for (int i = 0; i < max_iter; ++i) {
            double fx = f(x);
            double dfx = df(x);

            if (std::abs(dfx) < tol) {
                throw std::runtime_error("Derivative too small");
            }

            double x_new = x - fx / dfx;

            if (std::abs(x_new - x) < tol) {
                return x_new;
            }

            x = x_new;
        }

        return x;  // 返回当前估计值
    }

    double Numerical::Derivative(const std::function<double(double)>& f,
                                double x,
                                double h) {
        return (f(x + h) - f(x - h)) / (2 * h);
    }

} // namespace SimTools


// ============================================================
// 几何计算实现
// ============================================================

namespace SimTools {

    bool Geometry::IsPointInTriangle(const Point2D& point,
                                    const Point2D& a,
                                    const Point2D& b,
                                    const Point2D& c) {
        // 使用叉积判断
        double d1 = (b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x);
        double d2 = (c.x - b.x) * (point.y - b.y) - (c.y - b.y) * (point.x - b.x);
        double d3 = (a.x - c.x) * (point.y - c.y) - (a.y - c.y) * (point.x - c.x);

        bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

    bool Geometry::IsPointInPolygon(const Point2D& point,
                                   const std::vector<Point2D>& polygon) {
        // 射线法
        int intersections = 0;
        int n = static_cast<int>(polygon.size());

        for (int i = 0, j = n - 1; i < n; j = i++) {
            if (((polygon[i].y > point.y) != (polygon[j].y > point.y)) &&
                (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) /
                (polygon[j].y - polygon[i].y) + polygon[i].x)) {
                intersections++;
            }
        }

        return (intersections % 2) == 1;
    }

    double Geometry::DistanceToLineSegment(const Point2D& point,
                                          const Point2D& line_start,
                                          const Point2D& line_end) {
        double l2 = Distance(line_start, line_end) * Distance(line_start, line_end);

        if (l2 == 0) {
            return Distance(point, line_start);
        }

        double t = ((point.x - line_start.x) * (line_end.x - line_start.x) +
                   (point.y - line_start.y) * (line_end.y - line_start.y)) / l2;

        t = std::max(0.0, std::min(1.0, t));

        Point2D projection(
            line_start.x + t * (line_end.x - line_start.x),
            line_start.y + t * (line_end.y - line_start.y)
        );

        return Distance(point, projection);
    }

    Vector3d Geometry::PlaneNormal(const Vector3d& p1,
                                         const Vector3d& p2,
                                         const Vector3d& p3) {
        Vector3d v1 = p2 - p1;
        Vector3d v2 = p3 - p1;
        return v1.cross(v2).normalized();
    }

    double Geometry::DistanceToPlane(const Vector3d& point,
                                    const Vector3d& plane_point,
                                    const Vector3d& plane_normal) {
        return std::abs((point - plane_point).dot(plane_normal));
    }

} // namespace SimTools


// ============================================================
// 矩阵工具实现
// ============================================================

namespace SimTools {

    void MatrixUtils::Multiply(const double A[3][3], const double B[3], double C[3]) {
        for (int i = 0; i < 3; ++i) {
            C[i] = 0;
            for (int j = 0; j < 3; ++j) {
                C[i] += A[i][j] * B[j];
            }
        }
    }

    void MatrixUtils::Multiply3x3(const double A[3][3], const double B[3][3], double C[3][3]) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                C[i][j] = 0;
                for (int k = 0; k < 3; ++k) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
    }

    void MatrixUtils::Transpose(const double A[3][3], double B[3][3]) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                B[j][i] = A[i][j];
            }
        }
    }

    Matrix3d MatrixUtils::OuterProduct(const Vector3d& a,
                                         const Vector3d& b) {
        Matrix3d result;
        result(0, 0) = a[0] * b[0];
        result(0, 1) = a[0] * b[1];
        result(0, 2) = a[0] * b[2];
        result(1, 0) = a[1] * b[0];
        result(1, 1) = a[1] * b[1];
        result(1, 2) = a[1] * b[2];
        result(2, 0) = a[2] * b[0];
        result(2, 1) = a[2] * b[1];
        result(2, 2) = a[2] * b[2];
        return result;
    }

    Matrix3d MatrixUtils::SkewSymmetric(const Vector3d& v) {
        Matrix3d S;
        S(0, 0) = 0;      S(0, 1) = -v[2];  S(0, 2) = v[1];
        S(1, 0) = v[2];   S(1, 1) = 0;      S(1, 2) = -v[0];
        S(2, 0) = -v[1];  S(2, 1) = v[0];   S(2, 2) = 0;
        return S;
    }

    Matrix3d MatrixUtils::QuaternionToMatrix(const Vector4d& q) {
        Matrix3d R;
        double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

        R(0, 0) = 1 - 2*(q2*q2 + q3*q3);
        R(0, 1) = 2*(q1*q2 - q0*q3);
        R(0, 2) = 2*(q1*q3 + q0*q2);
        R(1, 0) = 2*(q1*q2 + q0*q3);
        R(1, 1) = 1 - 2*(q1*q1 + q3*q3);
        R(1, 2) = 2*(q2*q3 - q0*q1);
        R(2, 0) = 2*(q1*q3 - q0*q2);
        R(2, 1) = 2*(q2*q3 + q0*q1);
        R(2, 2) = 1 - 2*(q1*q1 + q2*q2);

        return R;
    }

    Vector3d MatrixUtils::MatrixToEuler(const Matrix3d& R) {
        Vector3d euler;
        euler[0] = std::atan2(R(2, 1), R(2, 2));  // roll
        euler[1] = std::atan2(-R(2, 0), std::sqrt(R(2, 1)*R(2, 1) + R(2, 2)*R(2, 2)));  // pitch
        euler[2] = std::atan2(R(1, 0), R(0, 0));  // yaw
        return euler;
    }

    Matrix3d MatrixUtils::EulerToMatrix(double roll, double pitch, double yaw) {
        double cr = std::cos(roll), sr = std::sin(roll);
        double cp = std::cos(pitch), sp = std::sin(pitch);
        double cy = std::cos(yaw), sy = std::sin(yaw);

        Matrix3d Rx;
        Rx(0, 0) = 1;   Rx(0, 1) = 0;   Rx(0, 2) = 0;
        Rx(1, 0) = 0;   Rx(1, 1) = cr;  Rx(1, 2) = sr;
        Rx(2, 0) = 0;   Rx(2, 1) = -sr; Rx(2, 2) = cr;

        Matrix3d Ry;
        Ry(0, 0) = cp;  Ry(0, 1) = 0;   Ry(0, 2) = -sp;
        Ry(1, 0) = 0;   Ry(1, 1) = 1;   Ry(1, 2) = 0;
        Ry(2, 0) = sp;  Ry(2, 1) = 0;   Ry(2, 2) = cp;

        Matrix3d Rz;
        Rz(0, 0) = cy;  Rz(0, 1) = sy;  Rz(0, 2) = 0;
        Rz(1, 0) = -sy; Rz(1, 1) = cy;  Rz(1, 2) = 0;
        Rz(2, 0) = 0;   Rz(2, 1) = 0;   Rz(2, 2) = 1;

        return Rz * Ry * Rx;
    }

} // namespace SimTools


// ============================================================
// 时间工具实现
// ============================================================

namespace SimTools {

    double Time::GetUnixTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration<double>(duration).count();
    }

    Time::GpsTime Time::UnixToGpsTime(double unix_time) {
        // GPS 时间始于 1980-01-06 00:00:00 UTC
        // Unix 时间始于 1970-01-01 00:00:00 UTC
        // 相差 315964800 秒
        constexpr double GPS_TO_UNIX_OFFSET = 315964800.0;

        double gps_seconds = unix_time - GPS_TO_UNIX_OFFSET;
        int week = static_cast<int>(gps_seconds / (7 * 24 * 3600));
        double seconds = gps_seconds - week * 7 * 24 * 3600;

        return {week, seconds};
    }

    double Time::GpsTimeToUnix(const GpsTime& gps_time) {
        constexpr double GPS_TO_UNIX_OFFSET = 315964800.0;
        return GPS_TO_UNIX_OFFSET + gps_time.week * 7 * 24 * 3600 + gps_time.seconds;
    }

} // namespace SimTools


// ============================================================
// 主函数示例
// ============================================================

// 只有在独立编译时才定义 main()
// 当此文件被其他程序包含时，不会编译这个 main()
#ifndef SIMTOOLS_SINGLE_FILE_NO_MAIN
int main() {
    using namespace SimTools;

    std::cout << "========================================" << std::endl;
    std::cout << "  SimTools v2.0 - 重构版本演示" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Version: " << Version::ToString() << std::endl;
    std::cout << std::endl;

    // ===== 示例 1: 坐标转换 =====
    std::cout << "1. 坐标转换测试:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    Coordinate::Vector3 beijing_gps(116.3974, 39.9093, 100.0);
    std::cout << "北京 GPS: " << beijing_gps.transpose() << std::endl;

    Coordinate::Vector3 beijing_ecef = Coordinate::GpsToEcef(beijing_gps);
    std::cout << "北京 ECEF: " << beijing_ecef.transpose() << std::endl;

    Coordinate::Vector3 beijing_back = Coordinate::EcefToGps(beijing_ecef);
    std::cout << "转换回 GPS: " << beijing_back.transpose() << std::endl;
    std::cout << std::endl;

    // ===== 示例 2: 地理计算 =====
    std::cout << "2. 地理计算测试:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    Coordinate::Vector3 shanghai_gps(121.4737, 31.2304, 0.0);
    double distance = Geodesy::GreatCircleDistance(
        beijing_gps[0], beijing_gps[1],
        shanghai_gps[0], shanghai_gps[1]
    );
    std::cout << "北京-上海距离: " << distance / 1000.0 << " km" << std::endl;

    double azimuth = Geodesy::Azimuth(beijing_gps, shanghai_gps);
    std::cout << "方位角: " << azimuth << " 度" << std::endl;
    std::cout << std::endl;

    // ===== 示例 3: 大气参数 =====
    std::cout << "3. 大气参数测试:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    double altitude = 10000.0;  // 10 km
    auto air = Atmosphere::GetParameters(altitude);
    std::cout << "高度: " << altitude << " m" << std::endl;
    std::cout << "温度: " << air.temperature << " K" << std::endl;
    std::cout << "气压: " << air.pressure << " Pa" << std::endl;
    std::cout << "密度: " << air.density << " kg/m³" << std::endl;
    std::cout << "声速: " << air.sound_speed << " m/s" << std::endl;
    std::cout << std::endl;

    // ===== 示例 4: 插值 =====
    std::cout << "4. 插值算法测试:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::vector<double> x_data = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> y_data = {0.0, 1.0, 4.0, 9.0, 16.0, 25.0};  // x²

    double x_query = 2.5;
    double y_linear = Interpolation::Linear(x_query, x_data, y_data);
    double y_lagrange = Interpolation::Lagrange7(x_query, x_data, y_data);

    std::cout << "在 x = " << x_query << " 处插值:" << std::endl;
    std::cout << "  线性插值: " << y_linear << std::endl;
    std::cout << "  拉格朗日插值: " << y_lagrange << std::endl;
    std::cout << "  真实值 (x²): " << x_query * x_query << std::endl;
    std::cout << std::endl;

    // ===== 示例 5: 随机数 =====
    std::cout << "5. 随机数测试:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    Random::Seed();
    std::cout << "均匀分布 [0, 1): " << Random::Uniform01() << std::endl;
    std::cout << "均匀分布 [10, 20]: " << Random::Uniform(10.0, 20.0) << std::endl;
    std::cout << "正态分布 N(0, 1): " << Random::Normal01() << std::endl;
    std::cout << "正态分布 N(100, 15): " << Random::Normal(100.0, 15.0) << std::endl;
    std::cout << std::endl;

    // ===== 示例 6: 数学工具 =====
    std::cout << "6. 数学工具测试:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    double angle = 370.0;
    std::cout << "角度规范化测试:" << std::endl;
    std::cout << "  " << angle << "° -> [-180, 180]: " << Math::Regulate180(angle) << "°" << std::endl;
    std::cout << "  " << angle << "° -> [0, 360]: " << Math::Regulate360(angle) << "°" << std::endl;
    std::cout << std::endl;

    // ===== 示例 7: 单位转换 =====
    std::cout << "7. 单位转换测试:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    double knots = 100.0;
    double ms = knots * Units::KNOTS_TO_MS;
    std::cout << knots << " knots = " << ms << " m/s" << std::endl;

    double km = 5.0;
    double nm = km * 1000.0 / Units::NM_TO_M;
    std::cout << km << " km = " << nm << " nautical miles" << std::endl;
    std::cout << std::endl;

    // ===== 示例 8: NED 坐标 =====
    std::cout << "8. NED 坐标测试:" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    Coordinate::Vector3 target_gps(116.5, 40.0, 0.0);
    Coordinate::Vector3 target_ned = Coordinate::EcefToNed(
        Coordinate::GpsToEcef(target_gps),
        beijing_gps
    );
    std::cout << "相对于北京的 NED 坐标: " << target_ned.transpose() << std::endl;
    std::cout << "  北: " << target_ned[0] << " m" << std::endl;
    std::cout << "  东: " << target_ned[1] << " m" << std::endl;
    std::cout << "  地: " << target_ned[2] << " m" << std::endl;
    std::cout << std::endl;

    std::cout << "========================================" << std::endl;
    std::cout << "  测试完成！" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
#endif // SIMTOOLS_SINGLE_FILE_NO_MAIN
