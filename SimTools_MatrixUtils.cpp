// ============================================================
// SimTools v2.0 - 矩阵工具模块实现
// ============================================================

#include "SimTools_v2.h"
#include <cmath>
#include <algorithm>

namespace SimTools {

    // ============================================================
    // C风格数组接口（兼容原代码）
    // ============================================================

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

    // ============================================================
    // Eigen 矩阵扩展
    // ============================================================

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

    // ============================================================
    // 四元数与旋转矩阵
    // ============================================================

    Matrix3d MatrixUtils::QuaternionToMatrix(const Quaterniond& q) {
        return q.ToRotationMatrix();
    }

    Quaterniond MatrixUtils::MatrixToQuaternion(const Matrix3d& R) {
        return Quaterniond::FromRotationMatrix(R);
    }

    Vector3d MatrixUtils::MatrixToEuler(const Matrix3d& R) {
        // 从旋转矩阵提取欧拉角 (ZYX 顺序，即 yaw-pitch-roll)
        Vector3d euler;

        // Pitch (绕 Y 轴) - 注意符号
        double pitch = std::asin(R(2, 0));

        // 检查万向锁情况
        if (std::abs(std::cos(pitch)) > 0.1) {
            // 正常情况
            euler[0] = std::atan2(-R(2, 1), R(2, 2));  // Roll
            euler[1] = pitch;                           // Pitch
            euler[2] = std::atan2(-R(1, 0), R(0, 0));  // Yaw
        } else {
            // 万向锁情况
            euler[0] = 0;  // Roll
            euler[1] = pitch;
            euler[2] = std::atan2(R(0, 1), R(1, 1));  // Yaw
        }

        return euler;
    }

    Matrix3d MatrixUtils::EulerToMatrix(double roll, double pitch, double yaw) {
        // 将角度转换为弧度
        double cr = std::cos(roll);
        double sr = std::sin(roll);
        double cp = std::cos(pitch);
        double sp = std::sin(pitch);
        double cy = std::cos(yaw);
        double sy = std::sin(yaw);

        // 分别构建绕 X, Y, Z 轴的旋转矩阵
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

        // 组合：R = Rz * Ry * Rx (ZYX 顺序)
        return Rz * Ry * Rx;
    }

    // ============================================================
    // 欧拉角与四元数转换
    // ============================================================

    Quaterniond MatrixUtils::EulerToQuaternion(double roll, double pitch, double yaw) {
        Eigen::Quaterniond q =
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        return Quaterniond(q.w(), q.x(), q.y(), q.z());
    }

    Vector3d MatrixUtils::QuaternionToEuler(const Quaterniond& q) {
        Eigen::Quaterniond eigen_q(q.W(), q.X(), q.Y(), q.Z());
        Eigen::Matrix3d R = eigen_q.toRotationMatrix();

        Vector3d euler;
        double pitch = std::asin(R(2, 0));

        if (std::abs(std::cos(pitch)) > 0.1) {
            euler[0] = std::atan2(-R(2, 1), R(2, 2));  // Roll
            euler[1] = pitch;                           // Pitch
            euler[2] = std::atan2(-R(1, 0), R(0, 0));  // Yaw
        } else {
            euler[0] = 0;  // Roll
            euler[1] = pitch;
            euler[2] = std::atan2(R(0, 1), R(1, 1));  // Yaw
        }

        return euler;
    }

    // ============================================================
    // 轴角与四元数转换
    // ============================================================

    Quaterniond MatrixUtils::AxisAngleToQuaternion(const Vector3d& axis, double angle) {
        Eigen::Vector3d eigen_axis(axis[0], axis[1], axis[2]);
        Eigen::Quaterniond q(Eigen::AngleAxisd(angle, eigen_axis.normalized()));
        return Quaterniond(q.w(), q.x(), q.y(), q.z());
    }

    void MatrixUtils::QuaternionToAxisAngle(const Quaterniond& q, Vector3d& axis, double& angle) {
        // 使用 Quaternion 类的方法
        q.ToAxisAngle(axis, angle);
    }

    // ============================================================
    // 四元数高级运算
    // ============================================================

    Quaterniond MatrixUtils::Slerp(const Quaterniond& q0, const Quaterniond& q1, double t) {
        // 使用 Quaternion 类的静态方法
        return Quaterniond::Slerp(q0, q1, t);
    }

    Vector3d MatrixUtils::RotateVector(const Quaterniond& q, const Vector3d& v) {
        // 使用 Quaternion 类的方法
        return q.Rotate(v);
    }

    Quaterniond MatrixUtils::QuaternionExp(const Vector3d& v) {
        // 使用 Quaternion 类的静态方法
        return Quaterniond::Exp(v);
    }

    Vector3d MatrixUtils::QuaternionLog(const Quaterniond& q) {
        // 使用 Quaternion 类的静态方法
        return Quaterniond::Log(q);
    }

} // namespace SimTools
