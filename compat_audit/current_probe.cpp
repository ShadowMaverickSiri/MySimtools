#include "SimTools_v2.h"
#include <iomanip>
#include <iostream>
#include <vector>

using namespace SimTools;

namespace {
void Scalar(const char* name, double value) {
    std::cout << name << ' ' << value << '\n';
}

void Vector(const char* name, const Vector3d& value) {
    std::cout << name << ' ' << value[0] << ' ' << value[1] << ' ' << value[2] << '\n';
}

void EmitMatrix(const char* name, const Matrix3d& value) {
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

    Vector3d norm_vec(3.0, -4.0, 12.0);
    Scalar("math.sign", Math::Sign(-7.0));
    Scalar("math.max", Math::Max(3.0, 7.0));
    Scalar("math.min", Math::Min(3.0, 7.0));
    Scalar("math.norm", Math::Norm2(norm_vec));
    Vector("math.normalize", Math::Normalize(norm_vec));
    Scalar("math.regulate_pi", Math::RegulatePi(4.0));
    Scalar("math.regulate_deg", Math::Regulate180(725.0));

    std::vector<double> x{0, 1, 2, 3, 4, 5, 6, 7};
    std::vector<double> y{1, 2, 5, 10, 17, 26, 37, 50};
    Scalar("interp.linear", Interpolation::Linear(2.25, x, y));
    Scalar("interp.lagrange7", Interpolation::Lagrange7(2.25, x, y));
    Scalar("interp.global", Interpolation::LagrangeGlobal(2.25, x, y));

    VectorXd ode_initial(1);
    ode_initial[0] = 1.0;
    VectorXd ode_result = Numerical::RungeKutta4(
        [](double, const VectorXd& state) { return state; },
        0.0, ode_initial, 0.1, 5);
    Scalar("numerical.rk4_autonomous", ode_result[0]);

    const double heights[] = {10, 1000, 10000, 11000, 20000, 32000, 47000, 51000};
    for (double height : heights) {
        Atmosphere::Parameters params = Atmosphere::GetParameters(height);
        std::cout << "atmosphere " << height << ' ' << params.pressure << ' '
                  << params.gravity << ' ' << params.sound_speed << ' ' << params.density << '\n';
        std::cout << "atmosphere.scalar " << height << ' '
                  << Atmosphere::Gravity(height) << ' '
                  << Atmosphere::SoundSpeed(height) << ' '
                  << Atmosphere::Density(height) << ' '
                  << Atmosphere::VelocityFromMach(2.0, height) << '\n';
    }

    const Vector3d gps_points[] = {
        Vector3d(0.0, 0.0, 0.0),
        Vector3d(116.3974, 39.9093, 100.0),
        Vector3d(-73.9857, -33.25, 1234.0)
    };
    for (int i = 0; i < 3; ++i) {
        Vector3d ecef = Coordinate::GpsToEcef(gps_points[i]);
        Vector((std::string("coordinate.ecef.") + std::to_string(i)).c_str(), ecef);
        Vector((std::string("coordinate.gps_back.") + std::to_string(i)).c_str(),
               Coordinate::EcefToGps(ecef));
    }

    EmitMatrix("coordinate.rotation.x", Coordinate::RotationMatrix(0.37, 1));
    EmitMatrix("coordinate.rotation.y", Coordinate::RotationMatrix(0.37, 2));
    EmitMatrix("coordinate.rotation.z", Coordinate::RotationMatrix(0.37, 3));
    EmitMatrix("coordinate.ecef_to_nue", Coordinate::EcefToNueMatrix(116.3974, 39.9093));
    EmitMatrix("coordinate.nue_to_ecef", Coordinate::NueToEcefMatrix(116.3974, 39.9093));

    const double rotation_angles[] = {
        -6.283185307179586, -3.141592653589793, -1.0, -0.1, 0.0,
        0.37, 1.5707963267948966, 3.141592653589793, 6.283185307179586
    };
    for (int axis = 1; axis <= 3; ++axis) {
        for (int i = 0; i < 9; ++i) {
            std::string key = "matrix.rotation_grid." + std::to_string(axis) + "." + std::to_string(i);
            EmitMatrix(key.c_str(), Coordinate::RotationMatrix(rotation_angles[i], axis));
        }
    }

    const double grid_longitudes[] = {-180.0, -120.0, -45.0, 0.0, 60.0, 116.3974, 179.9};
    const double grid_latitudes[] = {-90.0, -80.0, -30.0, 0.0, 45.0, 89.9, 90.0};
    for (int lon_index = 0; lon_index < 7; ++lon_index) {
        for (int lat_index = 0; lat_index < 7; ++lat_index) {
            std::string suffix = std::to_string(lon_index) + "." + std::to_string(lat_index);
            EmitMatrix(("matrix.ecef_to_nue_grid." + suffix).c_str(),
                       Coordinate::EcefToNueMatrix(grid_longitudes[lon_index],
                                                   grid_latitudes[lat_index]));
            EmitMatrix(("matrix.nue_to_ecef_grid." + suffix).c_str(),
                       Coordinate::NueToEcefMatrix(grid_longitudes[lon_index],
                                                   grid_latitudes[lat_index]));
        }
    }

    Vector3d velocity;
    Coordinate::VelocityToNue(0.3, 0.2, 250.0, velocity);
    Vector("coordinate.velocity_nue", velocity);
    Coordinate::NueToEcefVelocity(Vector3d(250.0, 0.0, 0.0), gps_points[1], velocity);
    Vector("coordinate.velocity_ecef_north", velocity);
    Vector("coordinate.relative_nue",
           Coordinate::EcefToNueMatrix(gps_points[1][0], gps_points[1][1]) *
           Vector3d(100.0, -200.0, 50.0));

    Scalar("geodesy.great_circle", Geodesy::GreatCircleDistance(122.24212, 33.25622, 135.12195, 40.10167));
    double distance, azimuth1, azimuth2;
    Geodesy::VincentyInverse(122.24212, 33.25622, 135.12195, 40.10167,
                            distance, azimuth1, azimuth2);
    std::cout << "geodesy.inverse " << distance << ' ' << azimuth1 << ' ' << azimuth2 << '\n';
    double lon2, lat2, direct_azimuth2;
    Geodesy::VincentyDirect(122.24212, 33.25622, azimuth1, distance,
                           lon2, lat2, direct_azimuth2);
    std::cout << "geodesy.direct " << lon2 << ' ' << lat2 << ' ' << direct_azimuth2 << '\n';

    Vector3d gps_a(122.24212, 33.25622, 100.0);
    Vector3d gps_b(122.5, 33.5, 200.0);
    Scalar("geodesy.site_distance", Geodesy::SiteDistance(gps_a, gps_b));
    Scalar("geodesy.site_azimuth_signed",
           Math::Regulate180(-Geodesy::SiteAzimuth(gps_a, gps_b)));
    Scalar("geodesy.site_azimuth_standard", Geodesy::SiteAzimuth(gps_a, gps_b));

    Scalar("geometry.triangle",
           Geometry::IsPointInTriangle(Geometry::Point2D(0.5, 0.75),
                                       Geometry::Point2D(0, 0),
                                       Geometry::Point2D(3, 0),
                                       Geometry::Point2D(0, 3)) ? 1.0 : 0.0);

    Scalar("angle.dms_to_decimal", Coordinate::DmsToDecimal(120, 30, 36));
    int degree, minute;
    double second;
    Coordinate::DecimalToDms(-12.345678, degree, minute, second);
    std::cout << "angle.decimal_to_dms " << degree << ' ' << minute << ' ' << second << '\n';

    double matrix_a[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 10}};
    double matrix_b[3] = {2, -1, 0.5};
    double matrix_c[3];
    MatrixUtils::Multiply(matrix_a, matrix_b, matrix_c);
    std::cout << "matrix.multiply " << matrix_c[0] << ' ' << matrix_c[1] << ' ' << matrix_c[2] << '\n';
    double matrix_t[3][3];
    MatrixUtils::Transpose(matrix_a, matrix_t);
    std::cout << "matrix.transpose";
    for (const auto& row : matrix_t) {
        for (double value : row) std::cout << ' ' << value;
    }
    std::cout << '\n';
    double matrix_d[3][3] = {{2, 0, 1}, {-1, 3, 2}, {4, 1, 0}};
    double matrix_product[3][3];
    MatrixUtils::Multiply3x3(matrix_a, matrix_d, matrix_product);
    std::cout << "matrix.multiply3x3";
    for (const auto& row : matrix_product) {
        for (double value : row) std::cout << ' ' << value;
    }
    std::cout << '\n';

    Scalar("index.find", Interpolation::FindIndex(2.0, std::vector<double>{0, 1, 2, 3, 4}));
    std::cout << "string.int " << FileIO::ToString(-12345) << '\n';
    Scalar("file.line_count", FileIO::CountLines("test_data.txt"));
    return 0;
}
