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

    Vector4d MatrixUtils::MatrixToQuaternion(const Matrix3d& R) {
        // Shepperd算法：数值稳定的旋转矩阵转四元数
        double trace = R.trace();

        Vector4d q;
        if (trace > 0) {
            double t = std::sqrt(trace + 1.0);
            q[0] = 0.5 * t;
            t = 0.5 / t;
            q[1] = (R(2, 1) - R(1, 2)) * t;
            q[2] = (R(0, 2) - R(2, 0)) * t;
            q[3] = (R(1, 0) - R(0, 1)) * t;
        } else {
            // 找出对角元素最大的
            int i = 0;
            if (R(1, 1) > R(0, 0)) i = 1;
            if (R(2, 2) > R(i, i)) i = 2;

            int j = (i + 1) % 3;
            int k = (j + 1) % 3;

            double t = std::sqrt(R(i, i) - R(j, j) - R(k, k) + 1.0);
            q[i + 1] = 0.5 * t;
            t = 0.5 / t;
            q[0] = (R(k, j) - R(j, k)) * t;
            q[j + 1] = (R(j, i) + R(i, j)) * t;
            q[k + 1] = (R(k, i) + R(i, k)) * t;
        }

        return q.normalized();
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

    Vector4d MatrixUtils::EulerToQuaternion(double roll, double pitch, double yaw) {
        // ZYX顺序 (yaw-pitch-roll)
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        return Vector4d(
            cr * cp * cy + sr * sp * sy,  // w
            sr * cp * cy - cr * sp * sy,  // x
            cr * sp * cy + sr * cp * sy,  // y
            cr * cp * sy - sr * sp * cy   // z
        );
    }

    Vector3d MatrixUtils::QuaternionToEuler(const Vector4d& q) {
        Vector4d qn = q.normalized();
        double w = qn[0], x = qn[1], y = qn[2], z = qn[3];

        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (w * y - z * x);
        double pitch;
        if (std::abs(sinp) >= 1) {
            pitch = std::copysign(Constants::PI / 2, sinp);
        } else {
            pitch = std::asin(sinp);
        }

        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return Vector3d(roll, pitch, yaw);
    }

    // ============================================================
    // 轴角与四元数转换
    // ============================================================

    Vector4d MatrixUtils::AxisAngleToQuaternion(const Vector3d& axis, double angle) {
        Vector3d n = axis.normalized();
        double half_angle = angle * 0.5;
        double sin_half = std::sin(half_angle);
        double cos_half = std::cos(half_angle);

        return Vector4d(
            cos_half,
            n[0] * sin_half,
            n[1] * sin_half,
            n[2] * sin_half
        );
    }

    void MatrixUtils::QuaternionToAxisAngle(const Vector4d& q, Vector3d& axis, double& angle) {
        Vector4d qn = q.normalized();
        double w = qn[0];
        angle = 2.0 * std::acos(std::clamp(w, -1.0, 1.0));

        double sin_half = std::sqrt(1.0 - w * w);
        if (sin_half > 1e-6) {
            axis = Vector3d(qn[1] / sin_half, qn[2] / sin_half, qn[3] / sin_half);
        } else {
            axis = Vector3d(1, 0, 0);
        }
    }

    // ============================================================
    // 四元数高级运算
    // ============================================================

    Vector4d MatrixUtils::Slerp(const Vector4d& q0, const Vector4d& q1, double t) {
        Vector4d q0n = q0.normalized();
        Vector4d q1n = q1.normalized();

        double dot = q0n.dot(q1n);

        // 选择最短路径
        if (dot < 0.0) {
            q1n = Vector4d(-q1n[0], -q1n[1], -q1n[2], -q1n[3]);
            dot = -dot;
        }

        // 四元数接近时使用线性插值
        if (dot > 0.9995) {
            return (q0n * (1.0 - t) + q1n * t).normalized();
        }

        double theta = std::acos(std::clamp(dot, -1.0, 1.0));
        double sin_theta = std::sin(theta);

        double scale0 = std::sin((1.0 - t) * theta) / sin_theta;
        double scale1 = std::sin(t * theta) / sin_theta;

        return q0n * scale0 + q1n * scale1;
    }

    Vector3d MatrixUtils::RotateVector(const Vector4d& q, const Vector3d& v) {
        Vector4d qn = q.normalized();
        Vector4d q_conj = qn.conjugate();
        Vector4d v_quat(0, v[0], v[1], v[2]);
        Vector4d result = qn * v_quat * q_conj;
        return Vector3d(result[1], result[2], result[3]);
    }

    Vector4d MatrixUtils::QuaternionExp(const Vector3d& v) {
        double norm = v.norm();
        if (norm < 1e-10) {
            return Vector4d::Identity();
        }

        double sin_norm = std::sin(norm);
        double cos_norm = std::cos(norm);

        return Vector4d(
            cos_norm,
            v[0] / norm * sin_norm,
            v[1] / norm * sin_norm,
            v[2] / norm * sin_norm
        );
    }

    Vector3d MatrixUtils::QuaternionLog(const Vector4d& q) {
        Vector4d qn = q.normalized();
        double w = std::clamp(qn[0], -1.0, 1.0);
        double theta = std::acos(w);

        if (std::abs(w) >= 1.0) {
            return Vector3d(0, 0, 0);
        }

        double sin_theta = std::sin(theta);
        double scale = theta / sin_theta;

        return Vector3d(qn[1] * scale, qn[2] * scale, qn[3] * scale);
    }

} // namespace SimTools


// ============================================================
// Vector4d 非inline成员函数实现
// ============================================================

namespace SimTools {

    // 从轴角创建四元数
    Vector4d Vector4d::FromAxisAngle(const Vector3d& axis, double angle) {
        Vector3d n = axis.normalized();
        double half_angle = angle * 0.5;
        double sin_half = std::sin(half_angle);
        double cos_half = std::cos(half_angle);

        return Vector4d(
            cos_half,
            n[0] * sin_half,
            n[1] * sin_half,
            n[2] * sin_half
        );
    }

    // 从欧拉角创建四元数
    Vector4d Vector4d::FromEuler(double roll, double pitch, double yaw) {
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        return Vector4d(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }

    // 用四元数旋转向量
    Vector3d Vector4d::rotate(const Vector3d& v) const {
        Vector4d qn = this->normalized();
        Vector4d q_conj = qn.conjugate();
        Vector4d v_quat(0, v[0], v[1], v[2]);
        Vector4d result = qn * v_quat * q_conj;
        return Vector3d(result[1], result[2], result[3]);
    }

    // 转换为欧拉角
    Vector3d Vector4d::toEuler() const {
        Vector4d qn = this->normalized();
        double w = qn[0], x = qn[1], y = qn[2], z = qn[3];

        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (w * y - z * x);
        double pitch;
        if (std::abs(sinp) >= 1) {
            pitch = std::copysign(Constants::PI / 2, sinp);
        } else {
            pitch = std::asin(sinp);
        }

        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return Vector3d(roll, pitch, yaw);
    }

    // 转换为轴角表示
    void Vector4d::toAxisAngle(Vector3d& axis, double& angle) const {
        Vector4d qn = this->normalized();
        double w = qn[0];
        angle = 2.0 * std::acos(std::clamp(w, -1.0, 1.0));

        double sin_half = std::sqrt(1.0 - w * w);
        if (sin_half > 1e-6) {
            axis = Vector3d(qn[1] / sin_half, qn[2] / sin_half, qn[3] / sin_half);
        } else {
            axis = Vector3d(1, 0, 0);
        }
    }

} // namespace SimTools
