#include "mini_test.h"
#include "matrix.h"
#include "matrix.pb.h"
#include "objects.h"
#include "tolerance.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Matrix", "Constructor") {
    // using session_cpp::Color;
    // using session_cpp::Matrix;

    const Matrix m = Matrix::zeros(2, 3);
    const Matrix eye = Matrix::identity(3);
    const Matrix ml = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    const Matrix mr = Matrix::from_rows({{1.0, 2.0}, {3.0, 4.0}});
    const Matrix mc = Matrix::from_cols({{1.0, 3.0}, {2.0, 4.0}});
    const double v00 = ml(0, 0);
    const double v01 = ml(0, 1);
    const double v10 = ml(1, 0);
    const double v11 = ml(1, 1);
    const bool eq = ml == mr;
    const bool ne = ml != Matrix::identity(2);
    const std::string sstr = m.str();
    const std::string srepr = eye.repr();
    const Matrix d = ml.duplicate();
    Matrix short_guid;
    short_guid.guid() = "id";
    const std::string short_repr = short_guid.repr();

    MINI_CHECK(m.rows == 2 && m.cols == 3);
    MINI_CHECK(m.name == "my_matrix" && !m.guid().empty());
    MINI_CHECK(eye(0, 0) == 1.0 && eye(1, 1) == 1.0 && eye(2, 2) == 1.0);
    MINI_CHECK(eye(0, 1) == 0.0);
    MINI_CHECK(v00 == 1.0 && v01 == 2.0 && v10 == 3.0 && v11 == 4.0);
    MINI_CHECK(mc == ml);
    MINI_CHECK(eq);
    MINI_CHECK(ne);
    MINI_CHECK(sstr.find("Matrix(2x3)") != std::string::npos);
    MINI_CHECK(srepr.find("Matrix(") != std::string::npos);
    MINI_CHECK(d == ml && d.guid() != ml.guid());
    MINI_CHECK(short_repr.find("guid='id...'") != std::string::npos);
    MINI_CHECK(m.mincolor == Color(11.0f / 255.0f, 25.0f / 255.0f, 87.0f / 255.0f));
    MINI_CHECK(m.maxcolor == Color(206.0f / 255.0f, 64.0f / 255.0f, 149.0f / 255.0f));
    MINI_CHECK(m.zerocolor == Color(185.0f / 255.0f, 185.0f / 255.0f, 189.0f / 255.0f));
    MINI_CHECK(m.textcolor == Color::white() && m.linecolor == Color::black());
    MINI_CHECK(m.cell == 1.0);
}

MINI_TEST("Matrix", "Properties") {
    // using session_cpp::Matrix;

    const Matrix m1 = Matrix::identity(3);
    const Matrix m2 = Matrix::zeros(2, 3);
    const Matrix m3 = Matrix::from_vec(3, 3, {1.0, 2.0, 3.0, 2.0, 5.0, 6.0, 3.0, 6.0, 9.0});
    const Matrix m4 = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    const bool sq1 = m1.is_square();
    const bool sq2 = m2.is_square();
    const bool sym1 = m3.is_symmetric();
    const bool sym2 = m4.is_symmetric();
    const double tr = m1.trace();

    MINI_CHECK(sq1);
    MINI_CHECK(!sq2);
    MINI_CHECK(sym1);
    MINI_CHECK(!sym2);
    MINI_CHECK(TOLERANCE.is_close(tr, 3.0));
}

MINI_TEST("Matrix", "Add") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    const Matrix b = Matrix::from_vec(2, 2, {5.0, 6.0, 7.0, 8.0});
    const Matrix c = a + b;

    MINI_CHECK(c(0, 0) == 6.0 && c(0, 1) == 8.0);
    MINI_CHECK(c(1, 0) == 10.0 && c(1, 1) == 12.0);
}

MINI_TEST("Matrix", "Subtract") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(2, 2, {5.0, 6.0, 7.0, 8.0});
    const Matrix b = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    const Matrix c = a - b;

    MINI_CHECK(c(0, 0) == 4.0 && c(0, 1) == 4.0);
    MINI_CHECK(c(1, 0) == 4.0 && c(1, 1) == 4.0);
}

MINI_TEST("Matrix", "Scale") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    const Matrix b = a * 2.0;
    const Matrix c = a * 3.0;

    MINI_CHECK(b(0, 0) == 2.0 && b(0, 1) == 4.0 && b(1, 0) == 6.0 && b(1, 1) == 8.0);
    MINI_CHECK(c(0, 0) == 3.0 && c(1, 1) == 12.0);
}

MINI_TEST("Matrix", "Multiply") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    const Matrix b = Matrix::from_vec(3, 2, {7.0, 8.0, 9.0, 10.0, 11.0, 12.0});
    const Matrix c = a * b;

    MINI_CHECK(c.rows == 2 && c.cols == 2);
    MINI_CHECK(TOLERANCE.is_close(c(0, 0), 58.0) && TOLERANCE.is_close(c(0, 1), 64.0));
    MINI_CHECK(TOLERANCE.is_close(c(1, 0), 139.0) && TOLERANCE.is_close(c(1, 1), 154.0));
}

MINI_TEST("Matrix", "Transpose") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    const Matrix t = a.transpose();

    MINI_CHECK(t.rows == 3 && t.cols == 2);
    MINI_CHECK(t(0, 0) == 1.0 && t(1, 0) == 2.0 && t(2, 0) == 3.0);
    MINI_CHECK(t(0, 1) == 4.0 && t(1, 1) == 5.0 && t(2, 1) == 6.0);
}

MINI_TEST("Matrix", "Determinant") {
    // using session_cpp::Matrix;

    const Matrix a1 = Matrix::from_vec(1, 1, {5.0});
    const Matrix a2 = Matrix::from_vec(2, 2, {4.0, 7.0, 2.0, 6.0});
    const Matrix a3 = Matrix::from_vec(3, 3, {1.0, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0});
    const Matrix eye3 = Matrix::identity(3);

    MINI_CHECK(TOLERANCE.is_close(a1.determinant(), 5.0));
    MINI_CHECK(TOLERANCE.is_close(a2.determinant(), 10.0));
    MINI_CHECK(TOLERANCE.is_close(a3.determinant(), 1.0));
    MINI_CHECK(TOLERANCE.is_close(eye3.determinant(), 1.0));
}

MINI_TEST("Matrix", "Inverse") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(2, 2, {4.0, 7.0, 2.0, 6.0});
    const std::optional<Matrix> inv = a.inverse();
    const Matrix singular = Matrix::from_vec(2, 2, {1.0, 2.0, 2.0, 4.0});
    const std::optional<Matrix> inv_none = singular.inverse();

    MINI_CHECK(inv.has_value());

    const Matrix prod = a * *inv;

    MINI_CHECK(TOLERANCE.is_close((*inv)(0, 0), 0.6) && TOLERANCE.is_close((*inv)(0, 1), -0.7));
    MINI_CHECK(TOLERANCE.is_close((*inv)(1, 0), -0.2) && TOLERANCE.is_close((*inv)(1, 1), 0.4));
    MINI_CHECK(!inv_none.has_value());
    MINI_CHECK(TOLERANCE.is_close(prod(0, 0), 1.0) && TOLERANCE.is_close(prod(1, 1), 1.0));
    MINI_CHECK(TOLERANCE.is_close(prod(0, 1), 0.0) && TOLERANCE.is_close(prod(1, 0), 0.0));
}

MINI_TEST("Matrix", "Solve") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(2, 2, {2.0, 1.0, 1.0, 3.0});
    const Matrix b = Matrix::from_vec(2, 1, {5.0, 10.0});
    const std::optional<Matrix> x = a.solve(b);

    MINI_CHECK(x.has_value());

    const double residual_0 = 2.0 * (*x)(0, 0) + 1.0 * (*x)(1, 0);
    const double residual_1 = 1.0 * (*x)(0, 0) + 3.0 * (*x)(1, 0);

    MINI_CHECK(TOLERANCE.is_close((*x)(0, 0), 1.0));
    MINI_CHECK(TOLERANCE.is_close((*x)(1, 0), 3.0));
    MINI_CHECK(TOLERANCE.is_close(residual_0, 5.0));
    MINI_CHECK(TOLERANCE.is_close(residual_1, 10.0));
}

MINI_TEST("Matrix", "Lu Decompose") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(3, 3, {2.0, 1.0, 1.0, 4.0, 3.0, 3.0, 8.0, 7.0, 9.0});
    Matrix lower;
    Matrix u;
    Matrix p;
    std::tie(lower, u, p) = a.lu_decompose();
    const Matrix pa = p * a;
    const Matrix lu = lower * u;

    MINI_CHECK(lower.rows == 3 && u.cols == 3);
    MINI_CHECK(TOLERANCE.is_close(pa(0, 0), lu(0, 0)) && TOLERANCE.is_close(pa(0, 1), lu(0, 1)));
    MINI_CHECK(TOLERANCE.is_close(pa(1, 0), lu(1, 0)) && TOLERANCE.is_close(pa(2, 2), lu(2, 2)));
    MINI_CHECK(TOLERANCE.is_close(lower(0, 1), 0.0) && TOLERANCE.is_close(lower(0, 2), 0.0));
    MINI_CHECK(TOLERANCE.is_close(lower(1, 2), 0.0));
}

MINI_TEST("Matrix", "Qr Decompose") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(3, 3, {12.0, -51.0, 4.0, 6.0, 167.0, -68.0, -4.0, 24.0, -41.0});
    Matrix q;
    Matrix r;
    std::tie(q, r) = a.qr_decompose();
    const Matrix qt = q.transpose();
    const Matrix qtq = qt * q;
    const Matrix qr_prod = q * r;

    MINI_CHECK(TOLERANCE.is_close(qtq(0, 0), 1.0));
    MINI_CHECK(TOLERANCE.is_close(qtq(1, 1), 1.0));
    MINI_CHECK(TOLERANCE.is_close(qtq(2, 2), 1.0));
    MINI_CHECK(TOLERANCE.is_close(qtq(0, 1), 0.0) && TOLERANCE.is_close(qtq(0, 2), 0.0));
    MINI_CHECK(TOLERANCE.is_close(qr_prod(0, 0), 12.0));
    MINI_CHECK(TOLERANCE.is_close(qr_prod(1, 1), 167.0));
    MINI_CHECK(TOLERANCE.is_close(qr_prod(2, 2), -41.0));
}

MINI_TEST("Matrix", "Cholesky") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(3, 3, {4.0, 2.0, 2.0, 2.0, 5.0, 3.0, 2.0, 3.0, 6.0});
    const std::optional<Matrix> lower = a.cholesky();

    MINI_CHECK(lower.has_value());

    const Matrix lt = lower->transpose();
    const Matrix llt = *lower * lt;
    const Matrix not_spd = Matrix::from_vec(2, 2, {1.0, 2.0, 2.0, 1.0});
    const std::optional<Matrix> l_none = not_spd.cholesky();

    MINI_CHECK(TOLERANCE.is_close(llt(0, 0), 4.0) && TOLERANCE.is_close(llt(0, 1), 2.0));
    MINI_CHECK(TOLERANCE.is_close(llt(1, 0), 2.0) && TOLERANCE.is_close(llt(1, 1), 5.0));
    MINI_CHECK(TOLERANCE.is_close(llt(2, 2), 6.0));
    MINI_CHECK(!l_none.has_value());
}

MINI_TEST("Matrix", "Eigenvalues") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(3, 3, {3.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0});
    std::vector<double> evs = a.eigenvalues();
    const std::vector<double> empty = Matrix().eigenvalues();

    std::sort(evs.begin(), evs.end());

    MINI_CHECK(evs.size() == 3);
    MINI_CHECK(TOLERANCE.is_close(evs[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(evs[1], 2.0));
    MINI_CHECK(TOLERANCE.is_close(evs[2], 3.0));
    MINI_CHECK(empty.empty());
}

MINI_TEST("Matrix", "Svd") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(3, 3, {1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0});
    std::vector<double> sv;
    std::tie(std::ignore, sv, std::ignore) = a.svd();

    std::sort(sv.begin(), sv.end(), std::greater<double>());

    MINI_CHECK(sv.size() == 3);
    MINI_CHECK(TOLERANCE.is_close(sv[0], 3.0));
    MINI_CHECK(TOLERANCE.is_close(sv[1], 2.0));
    MINI_CHECK(TOLERANCE.is_close(sv[2], 1.0));
}

MINI_TEST("Matrix", "Norms") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::from_vec(2, 2, {1.0, -2.0, 3.0, -4.0});
    const double nf = a.norm_frobenius();
    const double n1 = a.norm_1();
    const double ni = a.norm_inf();

    MINI_CHECK(TOLERANCE.is_close(nf, std::sqrt(30.0)));
    MINI_CHECK(TOLERANCE.is_close(n1, 6.0));
    MINI_CHECK(TOLERANCE.is_close(ni, 7.0));
}

MINI_TEST("Matrix", "Rank") {
    // using session_cpp::Matrix;

    const Matrix a = Matrix::identity(3);
    const Matrix b = Matrix::from_vec(3, 3, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0});
    const Matrix c = Matrix::zeros(3, 3);

    MINI_CHECK(a.rank() == 3);
    MINI_CHECK(b.rank() == 2);
    MINI_CHECK(c.rank() == 0);
}

MINI_TEST("Matrix", "Json Roundtrip") {
    // using session_cpp::Color;
    // using session_cpp::Matrix;

    Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    a.name = "test_matrix";
    a.mincolor = Color::blue();
    a.maxcolor = Color::red();
    a.zerocolor = Color::grey();
    a.textcolor = Color::black();
    a.linecolor = Color::white();
    a.cell = 2.5;

    a.file_json_dump("serialization/test_matrix.json");
    const Matrix loaded = Matrix::file_json_load("serialization/test_matrix.json");
    const Matrix parsed = Matrix::file_json_loads(a.file_json_dumps());

    MINI_CHECK(loaded.name == "test_matrix");
    MINI_CHECK(loaded.rows == 2 && loaded.cols == 3);
    MINI_CHECK(TOLERANCE.is_close(loaded(0, 0), 1.0) && TOLERANCE.is_close(loaded(1, 2), 6.0));
    MINI_CHECK(loaded.mincolor == Color::blue() && loaded.maxcolor == Color::red());
    MINI_CHECK(loaded.zerocolor == Color::grey() && loaded.textcolor == Color::black());
    MINI_CHECK(loaded.linecolor == Color::white() && loaded.cell == 2.5);
    MINI_CHECK(parsed == a);
}

MINI_TEST("Matrix", "Protobuf Roundtrip") {
    // using session_cpp::Color;
    // using session_cpp::Matrix;

    const Matrix fresh;
    const session_proto::Matrix fresh_proto = fresh.to_proto();
    Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    a.name = "test_matrix_proto";
    a.mincolor = Color::blue();
    a.maxcolor = Color::red();
    a.zerocolor = Color::grey();
    a.textcolor = Color::black();
    a.linecolor = Color::white();
    a.cell = 2.5;
    const std::string guid = a.guid();

    a.pb_dump("serialization/test_matrix.bin");
    const Matrix loaded = Matrix::pb_load("serialization/test_matrix.bin");
    const Matrix parsed = Matrix::pb_loads(a.pb_dumps());
    const Matrix converted = Matrix::from_proto(a.to_proto());

    MINI_CHECK(!fresh.has_guid());
    MINI_CHECK(fresh_proto.guid().empty());
    MINI_CHECK(loaded.name == "test_matrix_proto");
    MINI_CHECK(loaded.rows == 2 && loaded.cols == 3);
    MINI_CHECK(TOLERANCE.is_close(loaded(0, 0), 1.0) && TOLERANCE.is_close(loaded(1, 2), 6.0));
    MINI_CHECK(parsed == a);
    MINI_CHECK(converted == a);
    MINI_CHECK(loaded.guid() == guid && parsed.guid() == guid && converted.guid() == guid);
    MINI_CHECK(loaded.mincolor == Color::blue() && loaded.maxcolor == Color::red());
    MINI_CHECK(loaded.zerocolor == Color::grey() && loaded.textcolor == Color::black());
    MINI_CHECK(loaded.linecolor == Color::white() && loaded.cell == 2.5);
}

MINI_TEST("Matrix", "To Component") {
    // using session_cpp::Matrix;

    Matrix m = Matrix::from_rows({{1.0, -2.0}, {0.0, 3.0}});
    m.name = "stiffness";
    const Component component = m.to_component();
    const nlohmann::ordered_json& extra = component.extra;

    MINI_CHECK(component.type_name == "Matrix");
    MINI_CHECK(component.guid() == m.guid() && component.name == "stiffness");
    MINI_CHECK(extra.size() == 9);
    MINI_CHECK(extra.contains("cell") && extra.contains("zerocolor"));
    MINI_CHECK(!extra.contains("guid") && !extra.contains("name"));
    MINI_CHECK(!extra.contains("type"));
    MINI_CHECK(extra["data"].get<std::vector<double>>() == std::vector<double>({1.0, -2.0, 0.0, 3.0}));
}

MINI_TEST("Matrix", "From Component") {
    // using session_cpp::Color;
    // using session_cpp::Matrix;

    Matrix m = Matrix::from_rows({{1.0, -2.0}, {0.0, 3.0}});
    m.name = "stiffness";
    m.maxcolor = Color::red();
    m.cell = 0.5;
    const Matrix restored = Matrix::from_component(m.to_component());

    MINI_CHECK(restored == m);
    MINI_CHECK(restored.guid() == m.guid() && restored.name == "stiffness");
    MINI_CHECK(restored.maxcolor == Color::red() && restored.cell == 0.5);
}

MINI_TEST("Matrix", "Serialization Errors") {
    // using session_cpp::Matrix;

    const Matrix matrix;
    bool malformed_json = false;
    bool malformed_pb = false;
    bool json_write_failed = false;
    bool pb_write_failed = false;

    try {
        Matrix::file_json_loads("{}");
    } catch (const std::exception&) {
        malformed_json = true;
    }

    try {
        Matrix::pb_loads(std::string(1, static_cast<char>(0xff)));
    } catch (const std::runtime_error&) {
        malformed_pb = true;
    }

    try {
        matrix.file_json_dump("");
    } catch (const std::runtime_error&) {
        json_write_failed = true;
    }

    try {
        matrix.pb_dump("");
    } catch (const std::runtime_error&) {
        pb_write_failed = true;
    }

    MINI_CHECK(malformed_json);
    MINI_CHECK(malformed_pb);
    MINI_CHECK(json_write_failed);
    MINI_CHECK(pb_write_failed);
}

MINI_TEST("Matrix", "Shape Errors") {
    // using session_cpp::Matrix;

    bool negative = false;
    bool overflow = false;
    bool data_size = false;
    bool rows = false;
    bool cols = false;
    bool multiply = false;
    bool json = false;
    bool proto_negative = false;
    bool proto_data = false;

    try {
        Matrix(-1, 2);
    } catch (const std::invalid_argument&) {
        negative = true;
    }

    try {
        Matrix::from_vec(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), {});
    } catch (const std::invalid_argument&) {
        overflow = true;
    }

    try {
        Matrix::from_vec(2, 2, {1.0});
    } catch (const std::invalid_argument&) {
        data_size = true;
    }

    try {
        Matrix::from_rows({{1.0, 2.0}, {3.0}});
    } catch (const std::invalid_argument&) {
        rows = true;
    }

    try {
        Matrix::from_cols({{1.0, 2.0}, {3.0}});
    } catch (const std::invalid_argument&) {
        cols = true;
    }

    try {
        Matrix(2, 3) * Matrix(2, 2);
    } catch (const std::invalid_argument&) {
        multiply = true;
    }

    try {
        Matrix::file_json_loads(R"({"cols":2,"data":[1.0],"guid":"id","name":"bad","rows":2,"type":"Matrix"})");
    } catch (const std::invalid_argument&) {
        json = true;
    }

    session_proto::Matrix negative_proto;
    negative_proto.set_rows(-1);
    negative_proto.set_cols(2);

    try {
        Matrix::from_proto(negative_proto);
    } catch (const std::invalid_argument&) {
        proto_negative = true;
    }

    session_proto::Matrix data_proto;
    data_proto.set_rows(2);
    data_proto.set_cols(2);
    data_proto.add_data(1.0);

    try {
        Matrix::from_proto(data_proto);
    } catch (const std::invalid_argument&) {
        proto_data = true;
    }

    MINI_CHECK(negative);
    MINI_CHECK(overflow);
    MINI_CHECK(data_size);
    MINI_CHECK(rows);
    MINI_CHECK(cols);
    MINI_CHECK(multiply);
    MINI_CHECK(json);
    MINI_CHECK(proto_negative);
    MINI_CHECK(proto_data);
}

MINI_TEST("Matrix", "Tutorial Operations") {
    const Matrix a = Matrix::from_rows({{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
    const Matrix diagonal = Matrix::diag({2.0, 3.0});
    MINI_CHECK(diagonal == Matrix::from_rows({{2.0, 0.0}, {0.0, 3.0}}));
    MINI_CHECK(a.block(0, 1, 2, 2) == Matrix::from_rows({{2.0, 3.0}, {5.0, 6.0}}));
    MINI_CHECK(a.block(2, 3, 0, 0).data.empty());
    MINI_CHECK(a.scale_rows({2.0, 3.0}) == diagonal.matmul(a));
    MINI_CHECK(a.scale_cols({2.0, 3.0, 4.0}) == a.matmul(Matrix::diag({2.0, 3.0, 4.0})));
    MINI_CHECK(a.hadamard(a) == Matrix::from_rows({{1.0, 4.0, 9.0}, {16.0, 25.0, 36.0}}));
    MINI_CHECK(Matrix::from_rows({{3.0, 4.0}, {0.0, 0.0}}).row_norms() == Matrix::from_cols({{5.0, 0.0}}));
    MINI_CHECK(Matrix::zeros(2, 0).row_norms() == Matrix::zeros(2, 1));
    MINI_CHECK(a.matmul(Matrix::from_cols({{1.0, 0.0, -1.0}})) == Matrix::from_cols({{-2.0, -2.0}}));
    MINI_CHECK(a == a.transpose().transpose());
    MINI_CHECK(Matrix::diag({}).rows == 0);
}

MINI_TEST("Matrix", "Solve Multiple") {
    const Matrix a = Matrix::from_rows({{0.0, 2.0}, {1.0, 3.0}});
    const Matrix b = Matrix::from_rows({{8.0, 10.0, 12.0}, {13.0, 17.0, 21.0}});
    const Matrix expected = Matrix::from_rows({{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
    const auto x = a.solve(b);
    MINI_CHECK(x.has_value());
    MINI_CHECK(((*x) - expected).norm_inf() < 1e-12);
    MINI_CHECK((a.matmul(*x) - b).norm_inf() < 1e-12);
    const auto small = (Matrix::identity(2) * 1e-200).solve(expected * 1e-200);
    MINI_CHECK(small.has_value());
    MINI_CHECK(((*small) - expected).norm_inf() < 1e-12);
    MINI_CHECK(Matrix::zeros(0, 0).solve(Matrix::zeros(0, 3))->cols == 3);
}

MINI_TEST("Matrix", "Solve Positive Definite") {
    const Matrix a = Matrix::from_rows({{4.0, 1.0}, {1.0, 3.0}});
    const Matrix b = Matrix::from_rows({{8.0, 13.0, 18.0}, {13.0, 17.0, 21.0}});
    const Matrix expected = Matrix::from_rows({{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
    const auto x = a.solve_positive_definite(b);
    MINI_CHECK(x.has_value());
    MINI_CHECK(((*x) - expected).norm_inf() < 1e-12);
    MINI_CHECK((a.matmul(*x) - b).norm_inf() < 1e-12);
    const auto small = (Matrix::identity(2) * 1e-200).solve_positive_definite(expected * 1e-200);
    MINI_CHECK(small.has_value());
    MINI_CHECK(((*small) - expected).norm_inf() < 1e-12);
    MINI_CHECK(Matrix::zeros(0, 0).solve_positive_definite(Matrix::zeros(0, 3))->cols == 3);
}

MINI_TEST("Matrix", "Solver Errors") {
    const Matrix rhs = Matrix::identity(2);
    const Matrix singular = Matrix::from_rows({{1.0, 2.0}, {2.0, 4.0}});
    const Matrix indefinite = Matrix::from_rows({{1.0, 2.0}, {2.0, 1.0}});
    const Matrix asymmetric = Matrix::from_rows({{4.0, 1.0}, {0.0, 3.0}});
    MINI_CHECK(!singular.solve(rhs).has_value());
    MINI_CHECK(!singular.solve_positive_definite(rhs).has_value());
    MINI_CHECK(!indefinite.solve_positive_definite(rhs).has_value());
    MINI_CHECK(!asymmetric.solve_positive_definite(rhs).has_value());
    MINI_CHECK(!Matrix::zeros(2, 3).solve(rhs).has_value());
    MINI_CHECK(!rhs.solve(Matrix::zeros(3, 1)).has_value());
    MINI_CHECK(!rhs.solve(Matrix::zeros(2, 0)).has_value());
    MINI_CHECK(!rhs.solve_positive_definite(Matrix::zeros(3, 1)).has_value());
    MINI_CHECK(!rhs.solve_positive_definite(Matrix::zeros(2, 0)).has_value());
    const Matrix bad = Matrix::from_rows({{std::numeric_limits<double>::quiet_NaN(), 0.0}, {0.0, 1.0}});
    MINI_CHECK(!bad.solve(rhs).has_value());
    MINI_CHECK(!rhs.solve(bad).has_value());
    MINI_CHECK(!bad.solve_positive_definite(rhs).has_value());
    MINI_CHECK(!rhs.solve_positive_definite(bad).has_value());
}

MINI_TEST("Matrix", "Force Density") {
    const Matrix c_s = Matrix::from_rows({{1.0, -1.0, 0.0, 0.0, 0.0}, {1.0, 0.0, -1.0, 0.0, 0.0}, {1.0, 0.0, 0.0, -1.0, 0.0}, {1.0, 0.0, 0.0, 0.0, -1.0}});
    const Matrix c = c_s.block(0, 0, 4, 1);
    const Matrix cf = c_s.block(0, 1, 4, 4);
    const Matrix fixed = Matrix::from_rows({{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}});
    const Matrix loads = Matrix::from_rows({{0.0, 0.0, -1.0}});
    const Matrix d = c.transpose().matmul(c.scale_rows({1.0, 1.0, 1.0, 1.0}));
    const Matrix df = c.transpose().matmul(cf.scale_rows({1.0, 1.0, 1.0, 1.0}));
    const Matrix rhs = loads - df.matmul(fixed);
    const auto xyz = d.solve_positive_definite(rhs);
    MINI_CHECK(xyz.has_value());
    MINI_CHECK(((*xyz) - Matrix::from_rows({{0.0, 0.0, -0.25}})).norm_inf() < 1e-12);
    MINI_CHECK(((*d.solve(rhs)) - (*xyz)).norm_inf() < 1e-12);
    const Matrix uvw = c.matmul(*xyz) + cf.matmul(fixed);
    const Matrix lengths = uvw.row_norms();
    MINI_CHECK(std::abs(lengths(0, 0) - std::sqrt(2.0625)) < 1e-12);
    MINI_CHECK((c.transpose().matmul(uvw) - loads).norm_inf() < 1e-12);
    const Matrix reactions = cf.transpose().matmul(uvw);
    MINI_CHECK(std::abs(reactions(0, 2) + reactions(1, 2) + reactions(2, 2) + reactions(3, 2) - 1.0) < 1e-12);
}

MINI_TEST("Matrix", "Array Constructors") {
    const Matrix a = Matrix::array({{1.0, 2.0}, {3.0, 4.0}});
    MINI_CHECK(a == Matrix::from_rows({{1.0, 2.0}, {3.0, 4.0}}));
    MINI_CHECK(Matrix::column({2.0, 3.0}) == Matrix::from_cols({{2.0, 3.0}}));
    MINI_CHECK(Matrix::ones(2, 3) == Matrix::full(2, 3, 1.0));
    MINI_CHECK(Matrix::full(2, 2, -2.0).data == std::vector<double>({-2.0, -2.0, -2.0, -2.0}));
    MINI_CHECK(Matrix::eye(3) == Matrix::identity(3));
    MINI_CHECK(a.shape().first == 2 && a.shape().second == 2);
    MINI_CHECK(Matrix::column({}).rows == 0 && Matrix::column({}).cols == 1);
    MINI_CHECK(Matrix::full(0, 3, 2.0).data.empty());
}

MINI_TEST("Matrix", "Operation Errors") {
    const Matrix a = Matrix::ones(2, 3);
    const std::vector<std::function<void()>> invalid = {
        [&] { a.block(3, 0, 0, 0); }, [&] { a.block(1, 1, 2, 2); },
        [&] { a.scale_rows({1.0}); }, [&] { a.scale_cols({1.0}); },
        [&] { a.hadamard(Matrix::eye(2)); }, [&] { a.matmul(Matrix::eye(2)); }
    };
    for (const auto& operation : invalid) {
        bool rejected = false;
        try { operation(); } catch (const std::invalid_argument&) { rejected = true; }
        MINI_CHECK(rejected);
    }
}

} // namespace session_cpp
