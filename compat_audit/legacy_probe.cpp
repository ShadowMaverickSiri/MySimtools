#include "SimTools.h"
#include <iomanip>
#include <iostream>
#include <vector>

namespace {
class ExponentialOde : public SimTools {
public:
    void Equation(double, double state[], double derivative[], double*) override {
        derivative[0] = state[0];
    }
};

void Scalar(const char* name, double value) {
    std::cout << name << ' ' << value << '\n';
}

void Vector(const char* name, const Eigen::Vector3d& value) {
    std::cout << name << ' ' << value[0] << ' ' << value[1] << ' ' << value[2] << '\n';
}

void EmitMatrix(const char* name, const Eigen::Matrix3d& value) {
    std::cout << name;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            std::cout << ' ' << value(row, col);
        }
    }
    std::cout << '\n';
}
}

int main() {
    std::cout << std::setprecision(17);

    Eigen::Vector3d norm_vec(3.0, -4.0, 12.0);
    Scalar("math.sign", SimTools::sign(-7.0));
    Scalar("math.max", SimTools::Max(3.0, 7.0));
    Scalar("math.min", SimTools::Min(3.0, 7.0));
    Scalar("math.norm", SimTools::BoundNorm2(norm_vec));
    Vector("math.normalize", SimTools().Normalize(norm_vec));
    Scalar("math.regulate_pi", SimTools::Regulate180(4.0));
    Scalar("math.regulate_deg", SimTools::regulate(725.0));

    std::vector<double> x{0, 1, 2, 3, 4, 5, 6, 7};
    std::vector<double> y{1, 2, 5, 10, 17, 26, 37, 50};
    Scalar("interp.linear", SimTools::Interp2(2.25, x, y));
    Scalar("interp.lagrange7", SimTools::Interp_L7(2.25, x, y));
    Scalar("interp.global", SimTools::Insert_EL(2.25, x, y));

    ExponentialOde legacy_ode;
    double ode_state[1] = {1.0};
    double ode_history[6] = {};
    legacy_ode.RangKutta(0.0, ode_state, 1, 0.1, 6, ode_history, nullptr);
    Scalar("numerical.rk4_autonomous", ode_history[5]);

    const double heights[] = {10, 1000, 10000, 11000, 20000, 32000, 47000, 51000};
    for (double height : heights) {
        double pressure, gravity, sound_speed, density;
        SimTools::AirParaFromH(height, pressure, gravity, sound_speed, density);
        std::cout << "atmosphere " << height << ' ' << pressure << ' ' << gravity << ' '
                  << sound_speed << ' ' << density << '\n';
        std::cout << "atmosphere.scalar " << height << ' '
                  << SimTools::Caculate_g(height) << ' '
                  << SimTools::SoundSpeedFromH(height) << ' '
                  << SimTools::RhoFromH(height) << ' '
                  << SimTools::CalVFromMaH(2.0, height) << '\n';
    }

    const Eigen::Vector3d gps_points[] = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(116.3974, 39.9093, 100.0),
        Eigen::Vector3d(-73.9857, -33.25, 1234.0)
    };
    for (int i = 0; i < 3; ++i) {
        double gps_raw[3] = {gps_points[i][0], gps_points[i][1], gps_points[i][2]};
        double ecef_raw[3];
        SimTools::Gps2E(ecef_raw, gps_raw);
        Eigen::Vector3d ecef(ecef_raw[0], ecef_raw[1], ecef_raw[2]);
        Vector((std::string("coordinate.ecef.") + std::to_string(i)).c_str(), ecef);
        double lon, lat, height;
        SimTools::E2Gps(ecef[0], ecef[1], ecef[2], lon, lat, height);
        Vector((std::string("coordinate.gps_back.") + std::to_string(i)).c_str(),
               Eigen::Vector3d(lon, lat, height));
    }

    EmitMatrix("coordinate.rotation.x", SimTools::CoordinateTransM3(0.37, 1));
    EmitMatrix("coordinate.rotation.y", SimTools::CoordinateTransM3(0.37, 2));
    EmitMatrix("coordinate.rotation.z", SimTools::CoordinateTransM3(0.37, 3));
    EmitMatrix("coordinate.ecef_to_nue", SimTools::MakeE2NFromGps(116.3974, 39.9093));
    EmitMatrix("coordinate.nue_to_ecef", SimTools::MakeN2EFromGps(116.3974, 39.9093));

    const double rotation_angles[] = {
        -6.283185307179586, -3.141592653589793, -1.0, -0.1, 0.0,
        0.37, 1.5707963267948966, 3.141592653589793, 6.283185307179586
    };
    for (int axis = 1; axis <= 3; ++axis) {
        for (int i = 0; i < 9; ++i) {
            std::string key = "matrix.rotation_grid." + std::to_string(axis) + "." + std::to_string(i);
            EmitMatrix(key.c_str(), SimTools::CoordinateTransM3(rotation_angles[i], axis));
        }
    }

    const double grid_longitudes[] = {-180.0, -120.0, -45.0, 0.0, 60.0, 116.3974, 179.9};
    const double grid_latitudes[] = {-90.0, -80.0, -30.0, 0.0, 45.0, 89.9, 90.0};
    for (int lon_index = 0; lon_index < 7; ++lon_index) {
        for (int lat_index = 0; lat_index < 7; ++lat_index) {
            std::string suffix = std::to_string(lon_index) + "." + std::to_string(lat_index);
            EmitMatrix(("matrix.ecef_to_nue_grid." + suffix).c_str(),
                       SimTools::MakeE2NFromGps(grid_longitudes[lon_index],
                                               grid_latitudes[lat_index]));
            EmitMatrix(("matrix.nue_to_ecef_grid." + suffix).c_str(),
                       SimTools::MakeN2EFromGps(grid_longitudes[lon_index],
                                               grid_latitudes[lat_index]));
        }
    }

    Eigen::Vector3d velocity;
    SimTools::VnFromV(0.3, 0.2, 250.0, velocity);
    Vector("coordinate.velocity_nue", velocity);
    SimTools::VeFromV(250.0, gps_points[1], velocity);
    Vector("coordinate.velocity_ecef_north", velocity);
    SimTools::PnFromPe(Eigen::Vector3d(100.0, -200.0, 50.0), velocity,
                       gps_points[1][0], gps_points[1][1]);
    Vector("coordinate.relative_nue", velocity);

    Scalar("geodesy.great_circle", SimTools::BigCircle(122.24212, 33.25622, 135.12195, 40.10167));
    double distance, azimuth1, azimuth2;
    SimTools::Vincenty_inverse(122.24212, 33.25622, 135.12195, 40.10167,
                              distance, azimuth1, azimuth2);
    std::cout << "geodesy.inverse " << distance << ' ' << azimuth1 << ' ' << azimuth2 << '\n';
    double lon2, lat2, direct_azimuth2;
    SimTools::Vincenty(122.24212, 33.25622, azimuth1, distance,
                       lon2, lat2, direct_azimuth2);
    std::cout << "geodesy.direct " << lon2 << ' ' << lat2 << ' ' << direct_azimuth2 << '\n';

    Eigen::Vector3d gps_a(122.24212, 33.25622, 100.0);
    Eigen::Vector3d gps_b(122.5, 33.5, 200.0);
    Scalar("geodesy.site_distance", SimTools::GPS_R_Compute(gps_a, gps_b));
    Scalar("geodesy.site_azimuth_signed", SimTools::PhiL_Compute(gps_a, gps_b));

    SimTools::NODE_xy a{0, 0}, b{3, 0}, c{0, 3};
    double point[2] = {0.5, 0.75};
    Scalar("geometry.triangle", SimTools::Ifpointraingle(point, a, b, c) ? 1.0 : 0.0);

    Scalar("angle.dms_to_decimal", SimTools::Angle60To10(120, 30, 36));
    int degree, minute;
    double second;
    SimTools::Angle10To60(-12.345678, degree, minute, second);
    std::cout << "angle.decimal_to_dms " << degree << ' ' << minute << ' ' << second << '\n';

    double matrix_a[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 10}};
    double matrix_b[3] = {2, -1, 0.5};
    double matrix_c[3];
    SimTools::MatMul(matrix_a, matrix_b, matrix_c);
    std::cout << "matrix.multiply " << matrix_c[0] << ' ' << matrix_c[1] << ' ' << matrix_c[2] << '\n';
    double matrix_t[3][3];
    SimTools::MatTrans(matrix_a, matrix_t);
    std::cout << "matrix.transpose";
    for (const auto& row : matrix_t) {
        for (double value : row) std::cout << ' ' << value;
    }
    std::cout << '\n';
    double matrix_d[3][3] = {{2, 0, 1}, {-1, 3, 2}, {4, 1, 0}};
    double matrix_product[3][3];
    SimTools::MatMul33(matrix_a, matrix_d, matrix_product);
    std::cout << "matrix.multiply3x3";
    for (const auto& row : matrix_product) {
        for (double value : row) std::cout << ' ' << value;
    }
    std::cout << '\n';

    Scalar("index.find", SimTools::Index1(2.0, std::vector<double>{0, 1, 2, 3, 4}));
    std::cout << "string.int " << SimTools::NumberToString(-12345) << '\n';
    Scalar("file.line_count", SimTools::TxtRowcount(std::string("test_data.txt")));
    return 0;
}
