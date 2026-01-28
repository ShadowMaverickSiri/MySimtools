// ============================================================
// SimTools v2.0 - 数值计算模块实现
// ============================================================

#include "SimTools_v2.h"
#include <functional>
#include <cmath>
#include <stdexcept>

namespace SimTools {

    // ============================================================
    // 数值积分 - 龙格-库塔方法
    // ============================================================

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

    VectorXd Numerical::RungeKutta45(const OdeFunction& f,
                                           double t0,
                                           const VectorXd& y0,
                                           double t_end,
                                           double tolerance) {
        // 自适应步长 Runge-Kutta-Fehlberg 方法
        VectorXd y = y0;
        double t = t0;
        double h = (t_end - t0) / 100.0;  // 初始步长

        const double safety_factor = 0.9;
        const double min_step = 1e-10;
        const double max_step = (t_end - t0) / 10.0;

        while (t < t_end) {
            if (t + h > t_end) {
                h = t_end - t;
            }

            // RK4(5) 系数
            VectorXd k1 = f(t, y);
            VectorXd k2 = f(t + h / 4.0, y + h * k1 / 4.0);
            VectorXd k3 = f(t + 3.0 * h / 8.0, y + 3.0 * h / 32.0 * k1 + 9.0 * h / 32.0 * k2);
            VectorXd k4 = f(t + 12.0 * h / 13.0, y + 1932.0 * h / 2197.0 * k1 - 7200.0 * h / 2197.0 * k2 + 7296.0 * h / 2197.0 * k3);
            VectorXd k5 = f(t + h, y + 439.0 * h / 216.0 * k1 - 8.0 * h * k2 + 3680.0 * h / 513.0 * k3 - 845.0 * h / 4104.0 * k4);
            VectorXd k6 = f(t + h / 2.0, y - 8.0 * h / 27.0 * k1 + 2.0 * h * k2 - 3544.0 * h / 2565.0 * k3 + 1859.0 * h / 4104.0 * k4 - 11.0 * h / 40.0 * k5);

            // 4阶和5阶解
            VectorXd y4 = y + h * (25.0 / 216.0 * k1 + 1408.0 / 2565.0 * k3 + 2197.0 / 4104.0 * k4 - 1.0 / 5.0 * k5);
            VectorXd y5 = y + h * (16.0 / 135.0 * k1 + 6656.0 / 12825.0 * k3 + 28561.0 / 56430.0 * k4 - 9.0 / 50.0 * k5 + 2.0 / 55.0 * k6);

            // 误差估计
            VectorXd error = y5 - y4;
            double max_error = error.cwiseAbs().maxCoeff();

            // 调整步长
            if (max_error < tolerance) {
                // 接受步长
                y = y5;
                t += h;

                // 增大步长
                double scale = safety_factor * std::pow(tolerance / (max_error + 1e-10), 0.2);
                h = std::min(max_step, h * std::max(0.1, scale));
            } else {
                // 拒绝步长，缩小重试
                double scale = safety_factor * std::pow(tolerance / (max_error + 1e-10), 0.25);
                h = std::max(min_step, h * std::max(0.1, scale));
            }
        }

        return y;
    }

    // ============================================================
    // 根查找
    // ============================================================

    double Numerical::Bisection(const std::function<double(double)>& f,
                               double a, double b,
                               double tol) {
        double fa = f(a);
        double fb = f(b);

        if (fa * fb > 0) {
            throw std::runtime_error("Function has same sign at endpoints in Bisection");
        }

        double c;
        int max_iter = 1000;
        int iter = 0;

        while ((b - a) > tol && iter < max_iter) {
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
            iter++;
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
                throw std::runtime_error("Derivative too small in Newton method");
            }

            double x_new = x - fx / dfx;

            if (std::abs(x_new - x) < tol) {
                return x_new;
            }

            x = x_new;
        }

        return x;  // 返回当前估计值
    }

    // ============================================================
    // 数值微分
    // ============================================================

    double Numerical::Derivative(const std::function<double(double)>& f,
                                double x,
                                double h) {
        // 使用中心差分公式
        return (f(x + h) - f(x - h)) / (2 * h);
    }

} // namespace SimTools
