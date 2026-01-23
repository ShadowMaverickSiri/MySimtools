// ============================================================
// SimTools v2.0 - 仿真工具模块实现
// ============================================================

#include "SimTools_v2.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <fstream>
#include <mutex>

namespace SimTools {

    // ============================================================
    // 性能计时器实现
    // ============================================================

    class Simulation::Timer::Impl {
    public:
        std::chrono::high_resolution_clock::time_point start_time;
        std::chrono::high_resolution_clock::time_point end_time;
        bool running;

        Impl() : running(false) {
            start_time = std::chrono::high_resolution_clock::now();
            end_time = start_time;
        }
    };

    Simulation::Timer::Timer() : impl_(std::make_unique<Impl>()) {}

    Simulation::Timer::~Timer() = default;

    void Simulation::Timer::Start() {
        impl_->start_time = std::chrono::high_resolution_clock::now();
        impl_->running = true;
    }

    void Simulation::Timer::Stop() {
        impl_->end_time = std::chrono::high_resolution_clock::now();
        impl_->running = false;
    }

    double Simulation::Timer::ElapsedSeconds() const {
        auto end = impl_->running ? std::chrono::high_resolution_clock::now() : impl_->end_time;
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - impl_->start_time);
        return duration.count() / 1000000.0;
    }

    double Simulation::Timer::ElapsedMilliseconds() const {
        auto end = impl_->running ? std::chrono::high_resolution_clock::now() : impl_->end_time;
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - impl_->start_time);
        return duration.count() / 1000.0;
    }

    // ============================================================
    // 日志系统实现
    // ============================================================

    namespace {
        // 线程安全的日志状态
        static std::mutex log_mutex_;
        static Simulation::LogLevel current_log_level_ = Simulation::LogLevel::Info;
        static std::ofstream log_file_;
        static bool log_to_file_ = false;

        // 日志级别转字符串
        std::string LogLevelToString(Simulation::LogLevel level) {
            switch (level) {
                case Simulation::LogLevel::Debug:   return "DEBUG";
                case Simulation::LogLevel::Info:    return "INFO ";
                case Simulation::LogLevel::Warning: return "WARN ";
                case Simulation::LogLevel::Error:   return "ERROR";
                case Simulation::LogLevel::Fatal:   return "FATAL";
                default: return "UNKN ";
            }
        }

        // 获取当前时间戳
        std::string GetCurrentTimestamp() {
            auto now = std::chrono::system_clock::now();
            auto time_t_now = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;

            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S");
            ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
            return ss.str();
        }

        // 核心日志函数
        void WriteLog(Simulation::LogLevel level, const std::string& message) {
            std::lock_guard<std::mutex> lock(log_mutex_);

            if (level < current_log_level_) {
                return;
            }

            std::stringstream ss;
            ss << "[" << GetCurrentTimestamp() << "] "
               << "[" << LogLevelToString(level) << "] "
               << message;

            std::string log_message = ss.str();

            // 输出到控制台
            if (level >= Simulation::LogLevel::Error) {
                std::cerr << log_message << std::endl;
            } else {
                std::cout << log_message << std::endl;
            }

            // 输出到文件
            if (log_to_file_ && log_file_.is_open()) {
                log_file_ << log_message << std::endl;
                log_file_.flush();
            }
        }
    }

    void Simulation::Logger::Log(LogLevel level, const std::string& message) {
        WriteLog(level, message);
    }

    void Simulation::Logger::Debug(const std::string& message) {
        Log(LogLevel::Debug, message);
    }

    void Simulation::Logger::Info(const std::string& message) {
        Log(LogLevel::Info, message);
    }

    void Simulation::Logger::Warning(const std::string& message) {
        Log(LogLevel::Warning, message);
    }

    void Simulation::Logger::Error(const std::string& message) {
        Log(LogLevel::Error, message);
    }

    void Simulation::Logger::SetLogLevel(LogLevel level) {
        std::lock_guard<std::mutex> lock(log_mutex_);
        current_log_level_ = level;
    }

    void Simulation::Logger::SetOutputFile(const std::string& filename) {
        std::lock_guard<std::mutex> lock(log_mutex_);

        if (log_file_.is_open()) {
            log_file_.close();
        }

        if (!filename.empty()) {
            log_file_.open(filename, std::ios::out | std::ios::app);
            log_to_file_ = log_file_.is_open();
        } else {
            log_to_file_ = false;
        }
    }

} // namespace SimTools
