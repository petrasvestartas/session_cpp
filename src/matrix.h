#pragma once
#include "guid.h"
#include "json.h"
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace session_cpp {

/**
 * @class Matrix
 * @brief An NxM general-purpose matrix with row-major storage.
 */
class Matrix {
public:
    std::string name = "my_matrix";
    const std::string& guid() const { if (_guid.empty()) _guid = ::guid(); return _guid; }
    std::string& guid() { if (_guid.empty()) _guid = ::guid(); return _guid; }
    int rows = 0;
    int cols = 0;
    std::vector<double> data;

    Matrix() = default;
    Matrix(int rows, int cols);
    Matrix(const Matrix& other);
    Matrix& operator=(const Matrix& other);

    static Matrix zeros(int rows, int cols);
    static Matrix identity(int n);
    static Matrix from_vec(int rows, int cols, const std::vector<double>& data);
    static Matrix from_rows(const std::vector<std::vector<double>>& rows_list);
    static Matrix from_cols(const std::vector<std::vector<double>>& cols_list);

    double& operator()(int r, int c);
    const double& operator()(int r, int c) const;

    bool is_square() const;
    bool is_symmetric() const;
    double trace() const;

    Matrix duplicate() const;

    Matrix add(const Matrix& other) const;
    Matrix subtract(const Matrix& other) const;
    Matrix scale(double s) const;
    Matrix multiply(const Matrix& other) const;
    Matrix transpose() const;

    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(const Matrix& other) const;
    bool operator==(const Matrix& other) const;
    bool operator!=(const Matrix& other) const;

    std::tuple<Matrix, Matrix, Matrix, int> _lu_internal() const;
    std::tuple<Matrix, Matrix, Matrix> lu_decompose() const;
    double determinant() const;
    std::optional<Matrix> inverse() const;
    std::optional<Matrix> solve(const Matrix& b) const;
    std::tuple<Matrix, Matrix> qr_decompose() const;
    std::optional<Matrix> cholesky() const;
    std::vector<double> eigenvalues() const;
    std::vector<std::pair<double, std::vector<double>>> _eigen_decompose_symmetric() const;
    std::tuple<Matrix, std::vector<double>, Matrix> svd() const;

    double norm_frobenius() const;
    double norm_1() const;
    double norm_inf() const;
    int rank() const;

    std::string str() const;
    std::string repr() const;

    nlohmann::ordered_json jsondump() const;
    static Matrix jsonload(const nlohmann::json& data);
    std::string json_dumps() const;
    static Matrix json_loads(const std::string& json_string);
    void json_dump(const std::string& filename) const;
    static Matrix json_load(const std::string& filename);

    std::string pb_dumps() const;
    static Matrix pb_loads(const std::string& data);
    void pb_dump(const std::string& filename) const;
    static Matrix pb_load(const std::string& filename);

private:
    mutable std::string _guid; ///< Lazily generated unique identifier
};

} // namespace session_cpp
