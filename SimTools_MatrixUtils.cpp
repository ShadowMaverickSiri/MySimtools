// ============================================================
// SimTools v2.0 - 矩阵工具模块实现
// ============================================================

#include "SimTools_v2.h"
#include <cmath>

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

    Matrix3d MatrixUtils::QuaternionToMatrix(const Vector4d& q) {
        // 归一化四元数
        Vector4d q_normalized = q.normalized();

        double q0 = q_normalized[0];  // 标量部分
        double q1 = q_normalized[1];  // 向量部分 x
        double q2 = q_normalized[2];  // 向量部分 y
        double q3 = q_normalized[3];  // 向量部分 z

        // 四元数转旋转矩阵
        Matrix3d R;
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

} // namespace SimTools
