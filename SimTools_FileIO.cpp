// ============================================================
// SimTools v2.0 - 文件 I/O 模块实现
// ============================================================

#include "SimTools_v2.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <filesystem>

namespace SimTools {

    // ============================================================
    // 读取数据
    // ============================================================

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

            // 去除前后空白
            line.erase(0, line.find_first_not_of(" \t\r\n"));
            line.erase(line.find_last_not_of(" \t\r\n") + 1);

            if (line.empty()) {
                continue;
            }

            std::vector<double> row;
            std::stringstream ss(line);

            // 支持空格、逗号、制表符分隔
            char delimiter = ' ';
            if (line.find(',') != std::string::npos) {
                delimiter = ',';
            }

            std::string cell;
            while (std::getline(ss, cell, delimiter)) {
                // 去除单元格前后空白
                cell.erase(0, cell.find_first_not_of(" \t"));
                cell.erase(cell.find_last_not_of(" \t") + 1);

                if (!cell.empty()) {
                    try {
                        double value = std::stod(cell);
                        row.push_back(value);
                    } catch (...) {
                        // 忽略无法转换的单元格
                    }
                }
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

        // 检查所有行是否有相同的列数
        for (const auto& row : data) {
            if (static_cast<int>(row.size()) != cols) {
                // 行长度不一致，返回错误或使用最小列数
                cols = std::min(cols, static_cast<int>(row.size()));
            }
        }

        MatrixXd matrix(rows, cols);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < std::min(cols, static_cast<int>(data[i].size())); ++j) {
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

        if (matrix.cols() <= column_index || column_index < 0) {
            throw std::runtime_error("Column index out of range in file: " + filename);
        }

        std::vector<double> column;
        column.reserve(matrix.rows());

        for (int i = 0; i < matrix.rows(); ++i) {
            column.push_back(matrix(i, column_index));
        }

        return column;
    }

    // ============================================================
    // 写入数据
    // ============================================================

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

    // ============================================================
    // 文件信息
    // ============================================================

    int FileIO::CountLines(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return 0;
        }

        int count = 0;
        std::string line;

        while (std::getline(file, line)) {
            // 跳过空行和注释行
            if (!line.empty() && line[0] != '#') {
                // 去除空白后检查
                std::string trimmed = line;
                trimmed.erase(0, trimmed.find_first_not_of(" \t\r\n"));
                if (!trimmed.empty()) {
                    count++;
                }
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
            // 跳过空行和注释行
            if (!line.empty() && line[0] != '#') {
                std::stringstream ss(line);
                int count = 0;
                std::string cell;

                // 尝试多种分隔符
                char delimiter = ' ';
                if (line.find(',') != std::string::npos) {
                    delimiter = ',';
                }

                while (std::getline(ss, cell, delimiter)) {
                    cell.erase(0, cell.find_first_not_of(" \t"));
                    cell.erase(cell.find_last_not_of(" \t") + 1);
                    if (!cell.empty()) {
                        count++;
                    }
                }

                file.close();
                return count;
            }
        }

        file.close();
        return 0;
    }

    // ============================================================
    // 辅助功能
    // ============================================================

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
