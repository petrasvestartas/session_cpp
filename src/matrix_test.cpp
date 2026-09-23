#include "mini_test.h"
#include "matrix.h"
#include "matrix.pb.h"
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
}

MINI_TEST("Matrix", "Properties") {

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

    const Matrix a = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    const Matrix b = Matrix::from_vec(2, 2, {5.0, 6.0, 7.0, 8.0});
    const Matrix c = a + b;

    MINI_CHECK(c(0, 0) == 6.0 && c(0, 1) == 8.0);
    MINI_CHECK(c(1, 0) == 10.0 && c(1, 1) == 12.0);
}

MINI_TEST("Matrix", "Subtract") {

    const Matrix a = Matrix::from_vec(2, 2, {5.0, 6.0, 7.0, 8.0});
    const Matrix b = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    const Matrix c = a - b;

    MINI_CHECK(c(0, 0) == 4.0 && c(0, 1) == 4.0);
    MINI_CHECK(c(1, 0) == 4.0 && c(1, 1) == 4.0);
}

MINI_TEST("Matrix", "Scale") {

    const Matrix a = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    const Matrix b = a * 2.0;
    const Matrix c = a * 3.0;

    MINI_CHECK(b(0, 0) == 2.0 && b(0, 1) == 4.0 && b(1, 0) == 6.0 && b(1, 1) == 8.0);
    MINI_CHECK(c(0, 0) == 3.0 && c(1, 1) == 12.0);
}

MINI_TEST("Matrix", "Multiply") {

    const Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    const Matrix b = Matrix::from_vec(3, 2, {7.0, 8.0, 9.0, 10.0, 11.0, 12.0});
    const Matrix c = a * b;

    MINI_CHECK(c.rows == 2 && c.cols == 2);
    MINI_CHECK(TOLERANCE.is_close(c(0, 0), 58.0) && TOLERANCE.is_close(c(0, 1), 64.0));
    MINI_CHECK(TOLERANCE.is_close(c(1, 0), 139.0) && TOLERANCE.is_close(c(1, 1), 154.0));
}

MINI_TEST("Matrix", "Transpose") {

    const Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    const Matrix t = a.transpose();

    MINI_CHECK(t.rows == 3 && t.cols == 2);
    MINI_CHECK(t(0, 0) == 1.0 && t(1, 0) == 2.0 && t(2, 0) == 3.0);
    MINI_CHECK(t(0, 1) == 4.0 && t(1, 1) == 5.0 && t(2, 1) == 6.0);
}

MINI_TEST("Matrix", "Determinant") {

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

    const Matrix a = Matrix::from_vec(2, 2, {1.0, -2.0, 3.0, -4.0});
    const double nf = a.norm_frobenius();
    const double n1 = a.norm_1();
    const double ni = a.norm_inf();

    MINI_CHECK(TOLERANCE.is_close(nf, std::sqrt(30.0)));
    MINI_CHECK(TOLERANCE.is_close(n1, 6.0));
    MINI_CHECK(TOLERANCE.is_close(ni, 7.0));
}

MINI_TEST("Matrix", "Rank") {

    const Matrix a = Matrix::identity(3);
    const Matrix b = Matrix::from_vec(3, 3, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0});
    const Matrix c = Matrix::zeros(3, 3);

    MINI_CHECK(a.rank() == 3);
    MINI_CHECK(b.rank() == 2);
    MINI_CHECK(c.rank() == 0);
}

MINI_TEST("Matrix", "Json Roundtrip") {

    Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    a.name = "test_matrix";

    a.file_json_dump("serialization/test_matrix.json");
    const Matrix loaded = Matrix::file_json_load("serialization/test_matrix.json");
    const Matrix parsed = Matrix::file_json_loads(a.file_json_dumps());

    MINI_CHECK(loaded.name == "test_matrix");
    MINI_CHECK(loaded.rows == 2 && loaded.cols == 3);
    MINI_CHECK(TOLERANCE.is_close(loaded(0, 0), 1.0) && TOLERANCE.is_close(loaded(1, 2), 6.0));
    MINI_CHECK(parsed == a);
}

MINI_TEST("Matrix", "Protobuf Roundtrip") {

    const Matrix fresh;
    const session_proto::Matrix fresh_proto = fresh.to_proto();
    Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    a.name = "test_matrix_proto";
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
}

MINI_TEST("Matrix", "Serialization Errors") {

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

} // namespace session_cpp
