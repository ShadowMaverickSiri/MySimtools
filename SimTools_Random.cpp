// ============================================================
// SimTools v2.0 - 随机数生成模块实现
// ============================================================

#include "SimTools_v2.h"
#include <chrono>
#include <random>
#include <cmath>

namespace SimTools {

    // 线程局部的随机数生成器（C++11方式）
    thread_local static std::mt19937_64 generator_;
    thread_local static bool initialized_ = false;

    // 初始化随机数生成器
    static void InitializeGenerator() {
        if (!initialized_) {
            std::random_device rd;
            generator_.seed(rd());
            initialized_ = true;
        }
    }

    // ============================================================
    // 种子设置
    // ============================================================

    void Random::Seed(unsigned int seed) {
        if (seed == 0) {
            // 使用当前时间作为种子
            seed = static_cast<unsigned int>(
                std::chrono::system_clock::now().time_since_epoch().count()
            );
        }
        generator_.seed(seed);
        initialized_ = true;
    }

    // ============================================================
    // 均匀分布
    // ============================================================

    double Random::Uniform01() {
        InitializeGenerator();

        std::uniform_real_distribution<double> distribution(0.0, 1.0);
        return distribution(generator_);
    }

    double Random::Uniform(double a, double b) {
        InitializeGenerator();

        if (a > b) {
            std::swap(a, b);
        }

        std::uniform_real_distribution<double> distribution(a, b);
        return distribution(generator_);
    }

    int Random::UniformInt(int a, int b) {
        InitializeGenerator();

        if (a > b) {
            std::swap(a, b);
        }

        std::uniform_int_distribution<int> distribution(a, b);
        return distribution(generator_);
    }

    // ============================================================
    // 正态分布
    // ============================================================

    double Random::Normal01() {
        InitializeGenerator();

        std::normal_distribution<double> distribution(0.0, 1.0);
        return distribution(generator_);
    }

    double Random::Normal(double mu, double sigma) {
        InitializeGenerator();

        if (sigma < 0) {
            sigma = -sigma;
        }

        std::normal_distribution<double> distribution(mu, sigma);
        return distribution(generator_);
    }

    // ============================================================
    // 指数分布
    // ============================================================

    double Random::Exponential(double lambda) {
        InitializeGenerator();

        if (lambda <= 0) {
            lambda = 1.0;
        }

        std::exponential_distribution<double> distribution(lambda);
        return distribution(generator_);
    }

    // ============================================================
    // 韦伯分布
    // ============================================================

    double Random::Weibull(double shape, double scale) {
        InitializeGenerator();

        if (shape <= 0) {
            shape = 1.0;
        }
        if (scale <= 0) {
            scale = 1.0;
        }

        std::weibull_distribution<double> distribution(shape, scale);
        return distribution(generator_);
    }

} // namespace SimTools
