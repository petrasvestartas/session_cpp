#include "matrix.h"
#include "matrix.pb.h"
#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace session_cpp {

Matrix::Matrix(int rows, int cols) : rows(rows), cols(cols), data(rows * cols, 0.0) {}

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

Matrix Matrix::zeros(int rows, int cols) {
    return Matrix(rows, cols);
}

Matrix Matrix::identity(int n) {
    Matrix m(n, n);
    for (int i = 0; i < n; ++i) m(i, i) = 1.0;
    return m;
}

Matrix Matrix::from_vec(int rows, int cols, const std::vector<double>& d) {
    Matrix m(rows, cols);
    m.data = d;
    return m;
}

Matrix Matrix::from_rows(const std::vector<std::vector<double>>& rows_list) {
    int r = static_cast<int>(rows_list.size());
    int c = r > 0 ? static_cast<int>(rows_list[0].size()) : 0;
    Matrix m(r, c);
    for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j)
            m(i, j) = rows_list[i][j];
    return m;
}

Matrix Matrix::from_cols(const std::vector<std::vector<double>>& cols_list) {
    int c = static_cast<int>(cols_list.size());
    int r = c > 0 ? static_cast<int>(cols_list[0].size()) : 0;
    Matrix m(r, c);
    for (int j = 0; j < c; ++j)
        for (int i = 0; i < r; ++i)
            m(i, j) = cols_list[j][i];
    return m;
}

double& Matrix::operator()(int r, int c) {
    return data[r * cols + c];
}

const double& Matrix::operator()(int r, int c) const {
    return data[r * cols + c];
}

bool Matrix::is_square() const { return rows == cols; }

bool Matrix::is_symmetric() const {
    if (!is_square()) return false;
    for (int i = 0; i < rows; ++i)
        for (int j = i + 1; j < cols; ++j)
            if (std::abs((*this)(i, j) - (*this)(j, i)) > 1e-10) return false;
    return true;
}

double Matrix::trace() const {
    double s = 0.0;
    for (int i = 0; i < rows; ++i) s += (*this)(i, i);
    return s;
}

Matrix Matrix::duplicate() const {
    Matrix m = *this;

    return m;
}

std::string Matrix::str() const {
    return "Matrix(" + std::to_string(rows) + "x" + std::to_string(cols) + ")";
}

std::string Matrix::repr() const {
    std::string gp = guid().size() >= 8 ? guid().substr(0, 8) : guid();
    std::ostringstream oss;
    oss << "Matrix(name='" << name << "', guid='" << gp << "...', rows=" << rows << ", cols=" << cols << ", data=[";
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

Matrix Matrix::add(const Matrix& other) const {
    Matrix result(rows, cols);
    for (int i = 0; i < static_cast<int>(data.size()); ++i)
        result.data[i] = data[i] + other.data[i];
    return result;
}

Matrix Matrix::subtract(const Matrix& other) const {
    Matrix result(rows, cols);
    for (int i = 0; i < static_cast<int>(data.size()); ++i)
        result.data[i] = data[i] - other.data[i];
    return result;
}

Matrix Matrix::scale(double s) const {
    Matrix result(rows, cols);
    for (int i = 0; i < static_cast<int>(data.size()); ++i)
        result.data[i] = data[i] * s;
    return result;
}

Matrix Matrix::multiply(const Matrix& other) const {
    Matrix result(rows, other.cols);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < other.cols; ++j) {
            double s = 0.0;
            for (int k = 0; k < cols; ++k)
                s += (*this)(i, k) * other(k, j);
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

Matrix Matrix::operator+(const Matrix& other) const { return add(other); }
Matrix Matrix::operator-(const Matrix& other) const { return subtract(other); }
Matrix Matrix::operator*(const Matrix& other) const { return multiply(other); }

bool Matrix::operator==(const Matrix& other) const {
    if (rows != other.rows || cols != other.cols) return false;
    for (int i = 0; i < static_cast<int>(data.size()); ++i)
        if (std::abs(data[i] - other.data[i]) > 1e-10) return false;
    return true;
}

bool Matrix::operator!=(const Matrix& other) const { return !(*this == other); }

std::tuple<Matrix, Matrix, Matrix, int> Matrix::_lu_internal() const {
    int n = rows;
    Matrix u = *this;
    Matrix l = Matrix::identity(n);
    Matrix p = Matrix::identity(n);
    int swaps = 0;
    for (int k = 0; k < n; ++k) {
        double max_val = std::abs(u(k, k));
        int max_row = k;
        for (int i = k + 1; i < n; ++i)
            if (std::abs(u(i, k)) > max_val) { max_val = std::abs(u(i, k)); max_row = i; }
        if (max_row != k) {
            for (int j = 0; j < n; ++j) std::swap(u(k, j), u(max_row, j));
            for (int j = 0; j < n; ++j) std::swap(p(k, j), p(max_row, j));
            for (int j = 0; j < k; ++j) std::swap(l(k, j), l(max_row, j));
            ++swaps;
        }
        if (std::abs(u(k, k)) < 1e-14) continue;
        for (int i = k + 1; i < n; ++i) {
            double factor = u(i, k) / u(k, k);
            l(i, k) = factor;
            for (int j = k; j < n; ++j) u(i, j) -= factor * u(k, j);
        }
    }
    return {l, u, p, swaps};
}

std::tuple<Matrix, Matrix, Matrix> Matrix::lu_decompose() const {
    auto [l, u, p, _] = _lu_internal();
    return {l, u, p};
}

double Matrix::determinant() const {
    int n = rows;
    if (n == 1) return (*this)(0, 0);
    if (n == 2) return (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
    auto [_l, u, _p, swaps] = _lu_internal();
    double sign = (swaps % 2 == 0) ? 1.0 : -1.0;
    double prod = 1.0;
    for (int i = 0; i < n; ++i) prod *= u(i, i);
    return sign * prod;
}

std::optional<Matrix> Matrix::inverse() const {
    if (!is_square()) return std::nullopt;
    int n = rows;
    auto [l, u, p, _] = _lu_internal();
    for (int i = 0; i < n; ++i)
        if (std::abs(u(i, i)) < 1e-14) return std::nullopt;
    Matrix result(n, n);
    Matrix eye = Matrix::identity(n);
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
    int n = rows;
    auto [l, u, p, _] = _lu_internal();
    for (int i = 0; i < n; ++i)
        if (std::abs(u(i, i)) < 1e-14) return std::nullopt;
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
    int m = rows, n = cols;
    std::vector<std::vector<double>> a_cols(n, std::vector<double>(m));
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
        double norm = std::sqrt(std::inner_product(v.begin(), v.end(), v.begin(), 0.0));
        r(j, j) = norm;
        if (norm > 1e-14) {
            std::vector<double> q(m);
            for (int k = 0; k < m; ++k) q[k] = v[k] / norm;
            q_cols.push_back(q);
        } else {
            q_cols.push_back(std::vector<double>(m, 0.0));
        }
    }
    Matrix q = Matrix::zeros(m, n);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < m; ++i)
            q(i, j) = q_cols[j][i];
    return {q, r};
}

std::optional<Matrix> Matrix::cholesky() const {
    if (!is_square()) return std::nullopt;
    int n = rows;
    Matrix l(n, n);
    for (int i = 0; i < n; ++i) {
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
    }
    return l;
}

std::vector<double> Matrix::eigenvalues() const {
    int n = rows;
    Matrix a = *this;
    for (int iter = 0; iter < 1000 * n; ++iter) {
        auto [q, r] = a.qr_decompose();
        a = r.multiply(q);
        bool converged = true;
        for (int i = 0; i < n - 1; ++i)
            if (std::abs(a(i + 1, i)) >= 1e-10) { converged = false; break; }
        if (converged) break;
    }
    std::vector<double> ev(n);
    for (int i = 0; i < n; ++i) ev[i] = a(i, i);
    return ev;
}

std::vector<std::pair<double, std::vector<double>>> Matrix::_eigen_decompose_symmetric() const {
    int n = rows;
    Matrix a = *this;
    Matrix v = Matrix::identity(n);
    for (int iter = 0; iter < 1000 * n; ++iter) {
        auto [q, r] = a.qr_decompose();
        a = r.multiply(q);
        v = v.multiply(q);
        bool converged = true;
        for (int i = 0; i < n - 1; ++i)
            if (std::abs(a(i + 1, i)) >= 1e-10) { converged = false; break; }
        if (converged) break;
    }
    std::vector<std::pair<double, std::vector<double>>> pairs;
    for (int i = 0; i < n; ++i) {
        std::vector<double> evec(n);
        for (int j = 0; j < n; ++j) evec[j] = v(j, i);
        pairs.push_back({a(i, i), evec});
    }
    return pairs;
}

std::tuple<Matrix, std::vector<double>, Matrix> Matrix::svd() const {
    int m = rows, n = cols;
    Matrix at = transpose();
    Matrix ata = at.multiply(*this);
    auto pairs = ata._eigen_decompose_symmetric();
    std::sort(pairs.begin(), pairs.end(), [](const auto& a, const auto& b) { return b.first < a.first; });
    int k = std::min(m, n);
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
        if (sv[j] > 1e-12) {
            for (int i = 0; i < m; ++i) {
                double val = 0.0;
                for (int l = 0; l < n; ++l) val += (*this)(i, l) * v(l, j);
                u(i, j) = val / sv[j];
            }
        }
    }
    return {u, sv, v.transpose()};
}

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
    auto [u, sv, vt] = svd();
    if (sv.empty()) return 0;
    double max_sv = *std::max_element(sv.begin(), sv.end());
    double threshold = std::max(rows, cols) * max_sv * 1e-10;
    return static_cast<int>(std::count_if(sv.begin(), sv.end(), [threshold](double s) { return s > threshold; }));
}

///////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////

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
    Matrix m;
    m.guid() = d["guid"].get<std::string>();
    m.name = d["name"].get<std::string>();
    m.rows = d["rows"].get<int>();
    m.cols = d["cols"].get<int>();
    m.data = d["data"].get<std::vector<double>>();
    return m;
}

std::string Matrix::file_json_dumps() const { return jsondump().dump(); }

Matrix Matrix::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Matrix::file_json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(2);
}

Matrix Matrix::file_json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json d = nlohmann::json::parse(file);
    return jsonload(d);
}

///////////////////////////////////////////////////////////////////////////////
// Protobuf Serialization
///////////////////////////////////////////////////////////////////////////////

std::string Matrix::pb_dumps() const {
    session_proto::Matrix msg;
    msg.set_guid(guid());
    msg.set_name(name);
    msg.set_rows(rows);
    msg.set_cols(cols);
    for (double v : data) msg.add_data(v);
    return msg.SerializeAsString();
}

Matrix Matrix::pb_loads(const std::string& bytes) {
    session_proto::Matrix msg;
    msg.ParseFromString(bytes);
    Matrix m;
    m.guid() = msg.guid();
    m.name = msg.name();
    m.rows = msg.rows();
    m.cols = msg.cols();
    m.data.reserve(msg.data_size());
    for (int i = 0; i < msg.data_size(); ++i) m.data.push_back(msg.data(i));
    return m;
}

void Matrix::pb_dump(const std::string& filename) const {
    std::string d = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(d.data(), d.size());
}

Matrix Matrix::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string d((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return pb_loads(d);
}

} // namespace session_cpp
