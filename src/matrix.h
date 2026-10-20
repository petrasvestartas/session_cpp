#pragma once
#include "color.h"
#include "guid.h"
#include "json.h"
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace session_proto {
class Matrix;
}

namespace session_cpp {

class Component;

/// An NxM matrix with row-major storage.
class Matrix {
private:
    mutable std::string _guid; // Lazily minted GUID.

public:
    std::string name = "my_matrix"; // Matrix name.
    int rows = 0; // Row count.
    int cols = 0; // Column count.
    std::vector<double> data; // Row-major values.
    Color mincolor = Color(11.0f / 255.0f, 25.0f / 255.0f, 87.0f / 255.0f); // Cell colour at the smallest non-zero entry.
    Color maxcolor = Color(206.0f / 255.0f, 64.0f / 255.0f, 149.0f / 255.0f); // Cell colour at the largest non-zero entry.
    Color zerocolor = Color(185.0f / 255.0f, 185.0f / 255.0f, 189.0f / 255.0f); // Cell colour of a zero entry.
    Color textcolor = Color::white(); // Colour of the values written in the cells.
    Color linecolor = Color::black(); // Colour of the cell edges and the title.
    double cell = 1.0; // Edge length of one square cell.

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

    /// Construct a matrix from rows, like a two-dimensional NumPy array.
    static Matrix array(const std::vector<std::vector<double>>& values);

    /// Construct a column vector.
    static Matrix column(const std::vector<double>& values);

    /// Construct a constant matrix.
    static Matrix full(int rows, int cols, double value);

    /// Construct a matrix of ones.
    static Matrix ones(int rows, int cols);

    /// Construct an identity matrix.
    static Matrix eye(int n);

    /// Return (rows, cols).
    std::pair<int, int> shape() const;

    /// Construct an n x n identity matrix.
    static Matrix identity(int n);

    /// Construct from exact row-major data; throws when the size does not match.
    static Matrix from_vec(int rows, int cols, const std::vector<double>& data);

    /// Construct from equal-length rows.
    static Matrix from_rows(const std::vector<std::vector<double>>& rows_list);

    /// Construct from equal-length columns.
    static Matrix from_cols(const std::vector<std::vector<double>>& cols_list);

    /// Construct a square matrix with the given diagonal values.
    static Matrix diag(const std::vector<double>& values);

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

    /// Copy a rectangular block; indices are zero-based and sizes are counts.
    Matrix block(int row, int col, int rows, int cols) const;

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

    /// Compare dimensions, values within the matrix comparison tolerance, colours and cell size.
    bool operator==(const Matrix& other) const;

    /// Return whether two matrices differ.
    bool operator!=(const Matrix& other) const;

    // ═══════════════════════════════════════════════════════════════════════════
    // Linear algebra
    // ═══════════════════════════════════════════════════════════════════════════
    /// Multiply by a dimension-compatible matrix, including column vectors.
    Matrix matmul(const Matrix& other) const;

    /// Multiply matching entries of equal-sized matrices.
    Matrix hadamard(const Matrix& other) const;

    /// Scale each row without constructing a diagonal matrix.
    Matrix scale_rows(const std::vector<double>& values) const;

    /// Scale each column without constructing a diagonal matrix.
    Matrix scale_cols(const std::vector<double>& values) const;

    /// Return the Euclidean norm of each row as a column vector.
    Matrix row_norms() const;

    /// Return the transpose.
    Matrix transpose() const;

    /// Return (L, U, P) with P * A = L * U; throws unless square.
    std::tuple<Matrix, Matrix, Matrix> lu_decompose() const;

    /// Return the determinant; throws unless the matrix is square.
    double determinant() const;

    /// Return the inverse, or empty for a non-square or singular matrix.
    std::optional<Matrix> inverse() const;

    /// Solve A * X = B for one or more columns using partial pivoting; empty on invalid or singular input.
    std::optional<Matrix> solve(const Matrix& b) const;

    /// Solve a symmetric positive-definite system by Cholesky; empty on invalid input.
    std::optional<Matrix> solve_positive_definite(const Matrix& b) const;

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
    // Component
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a Session component of type Matrix with this guid and name, every other JSON field in extra.
    Component to_component() const;

    /// Construct from a component made by to_component, guid and name from the component.
    static Matrix from_component(const Component& component);

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
