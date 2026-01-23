// ============================================================
// SimTools v2.0 - 插值算法模块实现
// ============================================================

#include "SimTools_v2.h"
#include <stdexcept>

namespace SimTools {

    // ============================================================
    // 一维插值实现
    // ============================================================

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
        bool ascending = (xx[1] > xx[0]);

        if (ascending) {
            while (i < n && xx[i] < x) i++;
        } else {
            while (i < n && xx[i] > x) i++;
        }

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

    double Interpolation::CubicSpline(double x,
                                     const std::vector<double>& xx,
                                     const std::vector<double>& yy) {
        int n = static_cast<int>(xx.size());

        if (n < 2 || xx.size() != yy.size()) {
            return 0.0;
        }

        if (n == 2) {
            return Linear(x, xx, yy);
        }

        // 计算三次样条插值
        // 自然边界条件：二阶导数为0

        std::vector<double> h(n), alpha(n), l(n), mu(n), z(n), c(n), b(n), d(n);

        // 计算 h[i] = x[i+1] - x[i]
        for (int i = 0; i < n - 1; i++) {
            h[i] = xx[i + 1] - xx[i];
        }

        // 计算 alpha[i] = 3/h[i] * (y[i+1] - y[i]) - 3/h[i-1] * (y[i] - y[i-1])
        for (int i = 1; i < n - 1; i++) {
            alpha[i] = 3.0 / h[i] * (yy[i + 1] - yy[i]) -
                      3.0 / h[i - 1] * (yy[i] - yy[i - 1]);
        }

        // 初始化
        l[0] = 1.0;
        mu[0] = 0.0;
        z[0] = 0.0;

        // 计算中间值
        for (int i = 1; i < n - 1; i++) {
            l[i] = 2.0 * (xx[i + 1] - xx[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n - 1] = 1.0;
        z[n - 1] = 0.0;
        c[n - 1] = 0.0;

        // 回代求解
        for (int j = n - 2; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (yy[j + 1] - yy[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }

        // 找到 x 所在的区间
        int i = FindIndex(x, xx);
        if (i == 0) i = 1;
        if (i >= n) i = n - 1;

        double dx = x - xx[i - 1];
        return yy[i - 1] + b[i - 1] * dx + c[i - 1] * dx * dx + d[i - 1] * dx * dx * dx;
    }

    // ============================================================
    // 二维插值实现
    // ============================================================

    double Interpolation::Bilinear(double u, double v,
                                  const std::vector<double>& x,
                                  const std::vector<double>& y,
                                  const std::vector<std::vector<double>>& z) {
        if (x.empty() || y.empty() || z.empty()) {
            return 0.0;
        }

        auto [i, j] = FindIndex2D(u, v, x, y);

        // 边界处理
        if (i == 0) i = 1;
        if (i >= static_cast<int>(x.size())) i = static_cast<int>(x.size()) - 1;
        if (j == 0) j = 1;
        if (j >= static_cast<int>(y.size())) j = static_cast<int>(y.size()) - 1;

        // 双线性插值
        double x1 = x[i - 1], x2 = x[i];
        double y1 = y[j - 1], y2 = y[j];

        double q11 = z[j - 1][i - 1];
        double q12 = z[j][i - 1];
        double q21 = z[j - 1][i];
        double q22 = z[j][i];

        double f_x_y1 = q11 * (x2 - u) / (x2 - x1) + q21 * (u - x1) / (x2 - x1);
        double f_x_y2 = q12 * (x2 - u) / (x2 - x1) + q22 * (u - x1) / (x2 - x1);

        return f_x_y1 * (y2 - v) / (y2 - y1) + f_x_y2 * (v - y1) / (y2 - y1);
    }

    // ============================================================
    // 索引查找实现
    // ============================================================

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
