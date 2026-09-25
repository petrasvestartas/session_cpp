#pragma once
#include "guid.h"
#include "json.h"
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace session_proto {
class Matrix;
}

namespace session_cpp {

/// An NxM matrix with row-major storage.
class Matrix {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_matrix"; // Matrix name.
    int rows = 0; // Row count.
    int cols = 0; // Column count.
    std::vector<double> data; // Row-major values.

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty matrix.
    Matrix() = default;

    /// Construct a rows x cols matrix of zeros; throws for invalid dimensions.
    Matrix(int rows, int cols);

    /// Copy with a new guid and the same data.
    Matrix(const Matrix& other);

    /// Copy-assign with a new guid and the same data.
    Matrix& operator=(const Matrix& other);

    /// Move while preserving the guid.
    Matrix(Matrix&& other) noexcept = default;

    /// Move-assign while preserving the guid.
    Matrix& operator=(Matrix&& other) noexcept = default;

    /// Construct a rows x cols zero matrix.
    static Matrix zeros(int rows, int cols);

    /// Construct an n x n identity matrix.
    static Matrix identity(int n);

    /// Construct from exact row-major data; throws when the size does not match.
    static Matrix from_vec(int rows, int cols, const std::vector<double>& data);

    /// Construct from equal-length rows.
    static Matrix from_rows(const std::vector<std::vector<double>>& rows_list);

    /// Construct from equal-length columns.
    static Matrix from_cols(const std::vector<std::vector<double>>& cols_list);

    /// Copy with a new guid and the same data.
    Matrix duplicate() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    bool has_guid() const {
        return !_guid.empty();
    }

    /// Return the guid, creating it on first access.
    const std::string& guid() const {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable guid, creating it on first access.
    std::string& guid() {

        if (_guid.empty())
            _guid = ::guid();

        return _guid;
    }

    /// Return the mutable element at (row, col).
    double& operator()(int r, int c);

    /// Return the element at (row, col).
    const double& operator()(int r, int c) const;

    /// Return whether the matrix has equal row and column counts.
    bool is_square() const;

    /// Return whether the matrix is square and symmetric.
    bool is_symmetric() const;

    /// Return the diagonal sum; throws unless the matrix is square.
    double trace() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Operators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Add an equal-sized matrix.
    Matrix operator+(const Matrix& other) const;

    /// Subtract an equal-sized matrix.
    Matrix operator-(const Matrix& other) const;

    /// Multiply by a dimension-compatible matrix.
    Matrix operator*(const Matrix& other) const;

    /// Multiply every element by a scalar.
    Matrix operator*(double s) const;

    /// Compare dimensions and values within the matrix comparison tolerance.
    bool operator==(const Matrix& other) const;

    /// Return whether two matrices differ.
    bool operator!=(const Matrix& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Linear algebra
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the transpose.
    Matrix transpose() const;

    /// Return (L, U, P) with P * A = L * U; throws unless square.
    std::tuple<Matrix, Matrix, Matrix> lu_decompose() const;

    /// Return the determinant; throws unless the matrix is square.
    double determinant() const;

    /// Return the inverse, or empty for a non-square or singular matrix.
    std::optional<Matrix> inverse() const;

    /// Return x with A * x = b, or empty when no compatible unique solution exists.
    std::optional<Matrix> solve(const Matrix& b) const;

    /// Return (Q, R) from Gram-Schmidt decomposition.
    std::tuple<Matrix, Matrix> qr_decompose() const;

    /// Return lower L with A = L * L^T, or empty when not positive definite.
    std::optional<Matrix> cholesky() const;

    /// Return eigenvalues by bounded unshifted QR iteration; throws unless square.
    std::vector<double> eigenvalues() const;

    /// Return (U, singular values, V^T).
    std::tuple<Matrix, std::vector<double>, Matrix> svd() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Norms
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the Frobenius norm.
    double norm_frobenius() const;

    /// Return the maximum absolute column sum.
    double norm_1() const;

    /// Return the maximum absolute row sum.
    double norm_inf() const;

    /// Return the numerical rank.
    int rank() const;

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to an ordered JSON object.
    nlohmann::ordered_json jsondump() const;

    /// Deserialize from a JSON object.
    static Matrix jsonload(const nlohmann::json& data);

    /// Serialize to a JSON string.
    std::string file_json_dumps() const;

    /// Deserialize from a JSON string.
    static Matrix file_json_loads(const std::string& json_string);

    /// Write JSON to a file.
    void file_json_dump(const std::string& filename) const;

    /// Read JSON from a file.
    static Matrix file_json_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    session_proto::Matrix to_proto() const;

    /// Construct from a shape-valid protobuf message.
    static Matrix from_proto(const session_proto::Matrix& proto);

    /// Serialize to protobuf bytes.
    std::string pb_dumps() const;

    /// Deserialize from protobuf bytes.
    static Matrix pb_loads(const std::string& data);

    /// Write protobuf bytes to a file.
    void pb_dump(const std::string& filename) const;

    /// Read protobuf bytes from a file.
    static Matrix pb_load(const std::string& filename);

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the compact dimension string.
    std::string str() const;

    /// Return the detailed representation.
    std::string repr() const;

private:
    /// Return (L, U, P, swaps) by partial pivoting.
    std::tuple<Matrix, Matrix, Matrix, int> _lu_internal() const;

    /// Return (eigenvalue, eigenvector) pairs by QR iteration with accumulated Q.
    std::vector<std::pair<double, std::vector<double>>> _eigen_decompose_symmetric() const;
};

} // namespace session_cpp
