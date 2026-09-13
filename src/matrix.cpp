#include "matrix.h"
#include "matrix.pb.h"
#include "tolerance.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace session_cpp {

namespace {

constexpr double COMPARISON_TOLERANCE = Tolerance::ABSOLUTE / 10.0;
constexpr double PIVOT_TOLERANCE = Tolerance::ZERO_TOLERANCE / 100.0;
constexpr double SINGULAR_TOLERANCE = Tolerance::ZERO_TOLERANCE;

size_t matrix_size(int rows, int cols) {
    if (rows < 0 || cols < 0) throw std::invalid_argument("Matrix dimensions cannot be negative");
    const size_t row_count = static_cast<size_t>(rows);
    const size_t col_count = static_cast<size_t>(cols);
    if (row_count != 0 && col_count > std::vector<double>().max_size() / row_count)
        throw std::invalid_argument("Matrix dimensions are too large");
    return row_count * col_count;
}

int matrix_dimension(size_t value) {
    if (value > static_cast<size_t>(std::numeric_limits<int>::max()))
        throw std::invalid_argument("Matrix dimension is too large");
    return static_cast<int>(value);
}

}

Matrix::Matrix(int rows, int cols) : rows(rows), cols(cols), data(matrix_size(rows, cols), 0.0) {}

Matrix::Matrix(const Matrix& other)
    : name(other.name), rows(other.rows), cols(other.cols), data(other.data) {}

Matrix& Matrix::operator=(const Matrix& other) {
    if (this != &other) {
        _guid.clear();
        name = other.name;
        rows = other.rows;
        cols = other.cols;
        data = other.data;
    }
    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════

Matrix Matrix::zeros(int rows, int cols) {
    return Matrix(rows, cols);
}

Matrix Matrix::identity(int n) {
    Matrix m(n, n);
    for (int i = 0; i < n; ++i) m(i, i) = 1.0;
    return m;
}

Matrix Matrix::from_vec(int rows, int cols, const std::vector<double>& data) {
    if (data.size() != matrix_size(rows, cols))
        throw std::invalid_argument("Matrix data size does not match its dimensions");
    Matrix m(rows, cols);
    m.data = data;
    return m;
}

Matrix Matrix::from_rows(const std::vector<std::vector<double>>& rows_list) {
    const int r = matrix_dimension(rows_list.size());
    const int c = r > 0 ? matrix_dimension(rows_list[0].size()) : 0;
    for (const std::vector<double>& row : rows_list)
        if (row.size() != static_cast<size_t>(c))
            throw std::invalid_argument("Matrix rows must have equal lengths");
    Matrix m(r, c);
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            m(i, j) = rows_list[i][j];
    return m;
}

Matrix Matrix::from_cols(const std::vector<std::vector<double>>& cols_list) {
    const int c = matrix_dimension(cols_list.size());
    const int r = c > 0 ? matrix_dimension(cols_list[0].size()) : 0;
    for (const std::vector<double>& col : cols_list)
        if (col.size() != static_cast<size_t>(r))
            throw std::invalid_argument("Matrix columns must have equal lengths");
    Matrix m(r, c);
    for (int j = 0; j < c; ++j)
        for (int i = 0; i < r; ++i)
            m(i, j) = cols_list[j][i];
    return m;
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════

double& Matrix::operator()(int r, int c) {
    return data[static_cast<size_t>(r) * static_cast<size_t>(cols) + static_cast<size_t>(c)];
}

const double& Matrix::operator()(int r, int c) const {
    return data[static_cast<size_t>(r) * static_cast<size_t>(cols) + static_cast<size_t>(c)];
}

bool Matrix::is_square() const {
    return rows == cols;
}

bool Matrix::is_symmetric() const {
    if (!is_square()) return false;
    for (int i = 0; i < rows; ++i)
        for (int j = i + 1; j < cols; ++j)
            if (std::abs((*this)(i, j) - (*this)(j, i)) > COMPARISON_TOLERANCE) return false;
    return true;
}

double Matrix::trace() const {
    if (!is_square()) throw std::invalid_argument("Matrix trace requires a square matrix");
    double s = 0.0;
    for (int i = 0; i < rows; ++i) s += (*this)(i, i);
    return s;
}

Matrix Matrix::duplicate() const {
    return Matrix(*this);
}

// ═══════════════════════════════════════════════════════════════════════════
// Operations
// ═══════════════════════════════════════════════════════════════════════════

Matrix Matrix::add(const Matrix& other) const {
    if (rows != other.rows || cols != other.cols)
        throw std::invalid_argument("Matrix dimensions must match for addition");
    Matrix result(rows, cols);
    for (size_t i = 0; i < data.size(); ++i) result.data[i] = data[i] + other.data[i];
    return result;
}

Matrix Matrix::subtract(const Matrix& other) const {
    if (rows != other.rows || cols != other.cols)
        throw std::invalid_argument("Matrix dimensions must match for subtraction");
    Matrix result(rows, cols);
    for (size_t i = 0; i < data.size(); ++i) result.data[i] = data[i] - other.data[i];
    return result;
}

Matrix Matrix::scale(double s) const {
    Matrix result(rows, cols);
    for (size_t i = 0; i < data.size(); ++i) result.data[i] = data[i] * s;
    return result;
}

Matrix Matrix::multiply(const Matrix& other) const {
    if (cols != other.rows)
        throw std::invalid_argument("Matrix dimensions are incompatible for multiplication");
    Matrix result(rows, other.cols);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < other.cols; ++j) {
            double s = 0.0;
            for (int k = 0; k < cols; ++k) s += (*this)(i, k) * other(k, j);
            result(i, j) = s;
        }
    return result;
}

Matrix Matrix::transpose() const {
    Matrix result(cols, rows);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            result(j, i) = (*this)(i, j);
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════

Matrix Matrix::operator+(const Matrix& other) const {
    return add(other);
}

Matrix Matrix::operator-(const Matrix& other) const {
    return subtract(other);
}

Matrix Matrix::operator*(const Matrix& other) const {
    return multiply(other);
}

bool Matrix::operator==(const Matrix& other) const {
    if (rows != other.rows || cols != other.cols) return false;
    for (size_t i = 0; i < data.size(); ++i)
        if (std::abs(data[i] - other.data[i]) > COMPARISON_TOLERANCE) return false;
    return true;
}

bool Matrix::operator!=(const Matrix& other) const {
    return !(*this == other);
}

// ═══════════════════════════════════════════════════════════════════════════
// Linear algebra
// ═══════════════════════════════════════════════════════════════════════════

std::tuple<Matrix, Matrix, Matrix, int> Matrix::_lu_internal() const {
    const int n = rows;
    Matrix u = *this;
    Matrix l = Matrix::identity(n);
    Matrix p = Matrix::identity(n);
    int swaps = 0;
    for (int k = 0; k < n; ++k) {
        double max_val = std::abs(u(k, k));
        int max_row = k;
        for (int i = k + 1; i < n; ++i)
            if (std::abs(u(i, k)) > max_val) {
                max_val = std::abs(u(i, k));
                max_row = i;
            }
        if (max_row != k) {
            for (int j = 0; j < n; ++j) std::swap(u(k, j), u(max_row, j));
            for (int j = 0; j < n; ++j) std::swap(p(k, j), p(max_row, j));
            for (int j = 0; j < k; ++j) std::swap(l(k, j), l(max_row, j));
            ++swaps;
        }
        if (std::abs(u(k, k)) < PIVOT_TOLERANCE) continue;
        for (int i = k + 1; i < n; ++i) {
            const double factor = u(i, k) / u(k, k);
            l(i, k) = factor;
            for (int j = k; j < n; ++j) u(i, j) -= factor * u(k, j);
        }
    }
    return {l, u, p, swaps};
}

std::tuple<Matrix, Matrix, Matrix> Matrix::lu_decompose() const {
    if (!is_square()) throw std::invalid_argument("LU decomposition requires a square matrix");
    const auto [l, u, p, swaps] = _lu_internal();
    return {l, u, p};
}

double Matrix::determinant() const {
    if (!is_square()) throw std::invalid_argument("Matrix determinant requires a square matrix");
    const int n = rows;
    if (n == 1) return (*this)(0, 0);
    if (n == 2) return (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
    const auto [l, u, p, swaps] = _lu_internal();
    const double sign = swaps % 2 == 0 ? 1.0 : -1.0;
    double prod = 1.0;
    for (int i = 0; i < n; ++i) prod *= u(i, i);
    return sign * prod;
}

std::optional<Matrix> Matrix::inverse() const {
    if (!is_square()) return std::nullopt;
    const int n = rows;
    const auto [l, u, p, swaps] = _lu_internal();
    for (int i = 0; i < n; ++i)
        if (std::abs(u(i, i)) < PIVOT_TOLERANCE) return std::nullopt;
    Matrix result(n, n);
    const Matrix eye = Matrix::identity(n);
    for (int col = 0; col < n; ++col) {
        std::vector<double> pb(n, 0.0);
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                pb[i] += p(i, j) * eye(j, col);
        std::vector<double> y(n, 0.0);
        for (int i = 0; i < n; ++i) {
            y[i] = pb[i];
            for (int j = 0; j < i; ++j) y[i] -= l(i, j) * y[j];
        }
        std::vector<double> x(n, 0.0);
        for (int i = n - 1; i >= 0; --i) {
            x[i] = y[i];
            for (int j = i + 1; j < n; ++j) x[i] -= u(i, j) * x[j];
            x[i] /= u(i, i);
        }
        for (int i = 0; i < n; ++i) result(i, col) = x[i];
    }
    return result;
}

std::optional<Matrix> Matrix::solve(const Matrix& b) const {
    if (!is_square() || b.rows != rows || b.cols != 1) return std::nullopt;
    const int n = rows;
    const auto [l, u, p, swaps] = _lu_internal();
    for (int i = 0; i < n; ++i)
        if (std::abs(u(i, i)) < PIVOT_TOLERANCE) return std::nullopt;
    std::vector<double> pb(n, 0.0);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            pb[i] += p(i, j) * b(j, 0);
    std::vector<double> y(n, 0.0);
    for (int i = 0; i < n; ++i) {
        y[i] = pb[i];
        for (int j = 0; j < i; ++j) y[i] -= l(i, j) * y[j];
    }
    std::vector<double> x(n, 0.0);
    for (int i = n - 1; i >= 0; --i) {
        x[i] = y[i];
        for (int j = i + 1; j < n; ++j) x[i] -= u(i, j) * x[j];
        x[i] /= u(i, i);
    }
    Matrix result(n, 1);
    for (int i = 0; i < n; ++i) result(i, 0) = x[i];
    return result;
}

std::tuple<Matrix, Matrix> Matrix::qr_decompose() const {
    const int m = rows;
    const int n = cols;
    std::vector<std::vector<double>> a_cols(n, std::vector<double>(m, 0.0));
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < m; ++i)
            a_cols[j][i] = (*this)(i, j);
    std::vector<std::vector<double>> q_cols;
    Matrix r = Matrix::zeros(n, n);
    for (int j = 0; j < n; ++j) {
        std::vector<double> v = a_cols[j];
        for (int i = 0; i < j; ++i) {
            double rij = 0.0;
            for (int k = 0; k < m; ++k) rij += q_cols[i][k] * v[k];
            r(i, j) = rij;
            for (int k = 0; k < m; ++k) v[k] -= rij * q_cols[i][k];
        }
        double norm = 0.0;
        for (int k = 0; k < m; ++k) norm += v[k] * v[k];
        norm = std::sqrt(norm);
        r(j, j) = norm;
        std::vector<double> qcol(m, 0.0);
        if (norm > PIVOT_TOLERANCE)
            for (int k = 0; k < m; ++k) qcol[k] = v[k] / norm;
        q_cols.push_back(qcol);
    }
    Matrix q = Matrix::zeros(m, n);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < m; ++i)
            q(i, j) = q_cols[j][i];
    return {q, r};
}

std::optional<Matrix> Matrix::cholesky() const {
    if (!is_square()) return std::nullopt;
    const int n = rows;
    Matrix l(n, n);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j <= i; ++j) {
            double s = (*this)(i, j);
            for (int k = 0; k < j; ++k) s -= l(i, k) * l(j, k);
            if (i == j) {
                if (s <= 0.0) return std::nullopt;
                l(i, j) = std::sqrt(s);
            } else {
                l(i, j) = s / l(j, j);
            }
        }
    return l;
}

std::vector<double> Matrix::eigenvalues() const {
    if (!is_square()) throw std::invalid_argument("Matrix eigenvalues require a square matrix");
    const int n = rows;
    Matrix a = *this;
    for (int iter = 0; iter < 1000 * n; ++iter) {
        const auto [q, r] = a.qr_decompose();
        a = r.multiply(q);
        bool converged = true;
        for (int i = 1; i < n; ++i)
            if (std::abs(a(i, i - 1)) >= COMPARISON_TOLERANCE) {
                converged = false;
                break;
            }
        if (converged) break;
    }
    std::vector<double> ev(n, 0.0);
    for (int i = 0; i < n; ++i) ev[i] = a(i, i);
    return ev;
}

std::vector<std::pair<double, std::vector<double>>> Matrix::_eigen_decompose_symmetric() const {
    const int n = rows;
    Matrix a = *this;
    Matrix v = Matrix::identity(n);
    for (int iter = 0; iter < 1000 * n; ++iter) {
        const auto [q, r] = a.qr_decompose();
        a = r.multiply(q);
        v = v.multiply(q);
        bool converged = true;
        for (int i = 1; i < n; ++i)
            if (std::abs(a(i, i - 1)) >= COMPARISON_TOLERANCE) {
                converged = false;
                break;
            }
        if (converged) break;
    }
    std::vector<std::pair<double, std::vector<double>>> pairs;
    for (int i = 0; i < n; ++i) {
        std::vector<double> evec(n, 0.0);
        for (int j = 0; j < n; ++j) evec[j] = v(j, i);
        pairs.push_back({a(i, i), evec});
    }
    return pairs;
}

std::tuple<Matrix, std::vector<double>, Matrix> Matrix::svd() const {
    const int m = rows;
    const int n = cols;
    const Matrix at = transpose();
    const Matrix ata = at.multiply(*this);
    std::vector<std::pair<double, std::vector<double>>> pairs = ata._eigen_decompose_symmetric();
    std::sort(pairs.begin(), pairs.end(), [](const auto& a, const auto& b) { return b.first < a.first; });
    const int k = std::min(m, n);
    std::vector<double> sv;
    std::vector<std::vector<double>> v_cols;
    for (int i = 0; i < k; ++i) {
        sv.push_back(std::sqrt(std::max(0.0, pairs[i].first)));
        v_cols.push_back(pairs[i].second);
    }
    Matrix v = Matrix::zeros(n, k);
    for (int j = 0; j < k; ++j)
        for (int i = 0; i < n; ++i)
            v(i, j) = v_cols[j][i];
    Matrix u = Matrix::zeros(m, k);
    for (int j = 0; j < k; ++j) {
        if (sv[j] <= SINGULAR_TOLERANCE) continue;
        for (int i = 0; i < m; ++i) {
            double val = 0.0;
            for (int l = 0; l < n; ++l) val += (*this)(i, l) * v(l, j);
            u(i, j) = val / sv[j];
        }
    }
    return {u, sv, v.transpose()};
}

// ═══════════════════════════════════════════════════════════════════════════
// Norms
// ═══════════════════════════════════════════════════════════════════════════

double Matrix::norm_frobenius() const {
    double s = 0.0;
    for (double x : data) s += x * x;
    return std::sqrt(s);
}

double Matrix::norm_1() const {
    double max_sum = 0.0;
    for (int j = 0; j < cols; ++j) {
        double col_sum = 0.0;
        for (int i = 0; i < rows; ++i) col_sum += std::abs((*this)(i, j));
        if (col_sum > max_sum) max_sum = col_sum;
    }
    return max_sum;
}

double Matrix::norm_inf() const {
    double max_sum = 0.0;
    for (int i = 0; i < rows; ++i) {
        double row_sum = 0.0;
        for (int j = 0; j < cols; ++j) row_sum += std::abs((*this)(i, j));
        if (row_sum > max_sum) max_sum = row_sum;
    }
    return max_sum;
}

int Matrix::rank() const {
    const auto [u, sv, vt] = svd();
    if (sv.empty()) return 0;
    double max_sv = 0.0;
    for (double s : sv) max_sv = std::max(max_sv, s);
    const double threshold = std::max(rows, cols) * max_sv * COMPARISON_TOLERANCE;
    int count = 0;
    for (double s : sv)
        if (s > threshold) ++count;
    return count;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════

nlohmann::ordered_json Matrix::jsondump() const {
    nlohmann::ordered_json d;
    d["cols"] = cols;
    d["data"] = data;
    d["guid"] = guid();
    d["name"] = name;
    d["rows"] = rows;
    d["type"] = "Matrix";
    return d;
}

Matrix Matrix::jsonload(const nlohmann::json& d) {
    Matrix m = Matrix::from_vec(
        d["rows"].get<int>(),
        d["cols"].get<int>(),
        d["data"].get<std::vector<double>>()
    );
    m.guid() = d["guid"].get<std::string>();
    m.name = d["name"].get<std::string>();
    return m;
}

std::string Matrix::file_json_dumps() const {
    return jsondump().dump();
}

Matrix Matrix::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Matrix::file_json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file) throw std::runtime_error("Failed to open JSON file: " + filename);
    file << jsondump().dump(2);
    if (!file) throw std::runtime_error("Failed to write JSON file: " + filename);
}

Matrix Matrix::file_json_load(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) throw std::runtime_error("Failed to open JSON file: " + filename);
    return jsonload(nlohmann::json::parse(file));
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════

session_proto::Matrix Matrix::to_proto() const {
    session_proto::Matrix proto;
    if (has_guid()) proto.set_guid(guid());
    proto.set_name(name);
    proto.set_rows(rows);
    proto.set_cols(cols);
    for (double v : data) proto.add_data(v);
    return proto;
}

Matrix Matrix::from_proto(const session_proto::Matrix& proto) {
    Matrix matrix = Matrix::from_vec(
        proto.rows(),
        proto.cols(),
        std::vector<double>(proto.data().begin(), proto.data().end())
    );
    if (!proto.guid().empty()) matrix.guid() = proto.guid();
    matrix.name = proto.name();
    return matrix;
}

std::string Matrix::pb_dumps() const {
    return to_proto().SerializeAsString();
}

Matrix Matrix::pb_loads(const std::string& data) {
    session_proto::Matrix proto;
    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse Matrix protobuf data");
    return from_proto(proto);
}

void Matrix::pb_dump(const std::string& filename) const {
    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    if (!file) throw std::runtime_error("Failed to open protobuf file: " + filename);
    file.write(data.data(), data.size());
    if (!file) throw std::runtime_error("Failed to write protobuf file: " + filename);
}

Matrix Matrix::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) throw std::runtime_error("Failed to open protobuf file: " + filename);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (file.bad()) throw std::runtime_error("Failed to read protobuf file: " + filename);
    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════

std::string Matrix::str() const {
    return "Matrix(" + std::to_string(rows) + "x" + std::to_string(cols) + ")";
}

std::string Matrix::repr() const {
    std::ostringstream oss;
    oss << "Matrix(name='" << name << "', guid='" << guid().substr(0, 8)
        << "...', rows=" << rows << ", cols=" << cols << ", data=[";
    for (int i = 0; i < rows; ++i) {
        if (i > 0) oss << "; ";
        oss << "[";
        for (int j = 0; j < cols; ++j) {
            if (j > 0) oss << ", ";
            oss << std::fixed << (*this)(i, j);
        }
        oss << "]";
    }
    oss << "])";
    return oss.str();
}

} // namespace session_cpp
