#include "mini_test.h"
#include "matrix.h"
#include "tolerance.h"
#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Matrix", "Constructor") {
    // uncomment #include "matrix.h"
    Matrix m = Matrix::zeros(2, 3);
    Matrix eye = Matrix::identity(3);
    Matrix ml = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    Matrix mr = Matrix::from_rows({{1.0, 2.0}, {3.0, 4.0}});
    Matrix mc = Matrix::from_cols({{1.0, 3.0}, {2.0, 4.0}});
    double v00 = ml(0, 0);
    double v01 = ml(0, 1);
    double v10 = ml(1, 0);
    double v11 = ml(1, 1);
    bool eq = (ml == mr);
    bool ne = (ml != Matrix::identity(2));
    std::string sstr = m.str();
    std::string srepr = eye.repr();
    Matrix d = ml.duplicate();

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
}

MINI_TEST("Matrix", "Properties") {
    // uncomment #include "matrix.h"
    Matrix m1 = Matrix::identity(3);
    Matrix m2 = Matrix::zeros(2, 3);
    Matrix m3 = Matrix::from_vec(3, 3, {1.0, 2.0, 3.0, 2.0, 5.0, 6.0, 3.0, 6.0, 9.0});
    Matrix m4 = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    bool sq1 = m1.is_square();
    bool sq2 = m2.is_square();
    bool sym1 = m3.is_symmetric();
    bool sym2 = m4.is_symmetric();
    double tr = m1.trace();

    MINI_CHECK(sq1);
    MINI_CHECK(!sq2);
    MINI_CHECK(sym1);
    MINI_CHECK(!sym2);
    MINI_CHECK(TOLERANCE.is_close(tr, 3.0));
}

MINI_TEST("Matrix", "Add") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    Matrix b = Matrix::from_vec(2, 2, {5.0, 6.0, 7.0, 8.0});
    Matrix c = a.add(b);
    Matrix d = a + b;

    MINI_CHECK(c(0, 0) == 6.0 && c(0, 1) == 8.0);
    MINI_CHECK(c(1, 0) == 10.0 && c(1, 1) == 12.0);
    MINI_CHECK(c == d);
}

MINI_TEST("Matrix", "Subtract") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(2, 2, {5.0, 6.0, 7.0, 8.0});
    Matrix b = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    Matrix c = a.subtract(b);
    Matrix d = a - b;

    MINI_CHECK(c(0, 0) == 4.0 && c(0, 1) == 4.0);
    MINI_CHECK(c(1, 0) == 4.0 && c(1, 1) == 4.0);
    MINI_CHECK(c == d);
}

MINI_TEST("Matrix", "Scale") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(2, 2, {1.0, 2.0, 3.0, 4.0});
    Matrix b = a.scale(2.0);
    Matrix c = a.scale(3.0);

    MINI_CHECK(b(0, 0) == 2.0 && b(0, 1) == 4.0 && b(1, 0) == 6.0 && b(1, 1) == 8.0);
    MINI_CHECK(c(0, 0) == 3.0 && c(1, 1) == 12.0);
}

MINI_TEST("Matrix", "Multiply") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    Matrix b = Matrix::from_vec(3, 2, {7.0, 8.0, 9.0, 10.0, 11.0, 12.0});
    Matrix c = a.multiply(b);
    Matrix d = a * b;

    MINI_CHECK(c.rows == 2 && c.cols == 2);
    MINI_CHECK(TOLERANCE.is_close(c(0, 0), 58.0) && TOLERANCE.is_close(c(0, 1), 64.0));
    MINI_CHECK(TOLERANCE.is_close(c(1, 0), 139.0) && TOLERANCE.is_close(c(1, 1), 154.0));
    MINI_CHECK(c == d);
}

MINI_TEST("Matrix", "Transpose") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    Matrix t = a.transpose();

    MINI_CHECK(t.rows == 3 && t.cols == 2);
    MINI_CHECK(t(0, 0) == 1.0 && t(1, 0) == 2.0 && t(2, 0) == 3.0);
    MINI_CHECK(t(0, 1) == 4.0 && t(1, 1) == 5.0 && t(2, 1) == 6.0);
}

MINI_TEST("Matrix", "Determinant") {
    // uncomment #include "matrix.h"
    Matrix a1 = Matrix::from_vec(1, 1, {5.0});
    Matrix a2 = Matrix::from_vec(2, 2, {4.0, 7.0, 2.0, 6.0});
    Matrix a3 = Matrix::from_vec(3, 3, {1.0, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0});
    Matrix eye3 = Matrix::identity(3);

    MINI_CHECK(TOLERANCE.is_close(a1.determinant(), 5.0));
    MINI_CHECK(TOLERANCE.is_close(a2.determinant(), 10.0));
    MINI_CHECK(TOLERANCE.is_close(a3.determinant(), 1.0));
    MINI_CHECK(TOLERANCE.is_close(eye3.determinant(), 1.0));
}

MINI_TEST("Matrix", "Inverse") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(2, 2, {4.0, 7.0, 2.0, 6.0});
    auto inv = a.inverse();
    Matrix singular = Matrix::from_vec(2, 2, {1.0, 2.0, 2.0, 4.0});
    auto inv_none = singular.inverse();
    Matrix prod = a.multiply(*inv);

    MINI_CHECK(inv.has_value());
    MINI_CHECK(TOLERANCE.is_close((*inv)(0, 0), 0.6) && TOLERANCE.is_close((*inv)(0, 1), -0.7));
    MINI_CHECK(TOLERANCE.is_close((*inv)(1, 0), -0.2) && TOLERANCE.is_close((*inv)(1, 1), 0.4));
    MINI_CHECK(!inv_none.has_value());
    MINI_CHECK(TOLERANCE.is_close(prod(0, 0), 1.0) && TOLERANCE.is_close(prod(1, 1), 1.0));
    MINI_CHECK(TOLERANCE.is_close(prod(0, 1), 0.0) && TOLERANCE.is_close(prod(1, 0), 0.0));
}

MINI_TEST("Matrix", "Solve") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(2, 2, {2.0, 1.0, 1.0, 3.0});
    Matrix b = Matrix::from_vec(2, 1, {5.0, 10.0});
    auto x = a.solve(b);
    // 2x+y=5, x+3y=10 → x=1, y=3

    MINI_CHECK(x.has_value());
    double residual_0 = 2.0 * (*x)(0, 0) + 1.0 * (*x)(1, 0);
    double residual_1 = 1.0 * (*x)(0, 0) + 3.0 * (*x)(1, 0);

    MINI_CHECK(TOLERANCE.is_close((*x)(0, 0), 1.0));
    MINI_CHECK(TOLERANCE.is_close((*x)(1, 0), 3.0));
    MINI_CHECK(TOLERANCE.is_close(residual_0, 5.0));
    MINI_CHECK(TOLERANCE.is_close(residual_1, 10.0));
}

MINI_TEST("Matrix", "Lu Decompose") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(3, 3, {2.0, 1.0, 1.0, 4.0, 3.0, 3.0, 8.0, 7.0, 9.0});
    auto [l, u, p] = a.lu_decompose();
    Matrix pa = p.multiply(a);
    Matrix lu = l.multiply(u);

    MINI_CHECK(l.rows == 3 && u.cols == 3);
    MINI_CHECK(TOLERANCE.is_close(pa(0, 0), lu(0, 0)) && TOLERANCE.is_close(pa(0, 1), lu(0, 1)));
    MINI_CHECK(TOLERANCE.is_close(pa(1, 0), lu(1, 0)) && TOLERANCE.is_close(pa(2, 2), lu(2, 2)));
    MINI_CHECK(TOLERANCE.is_close(l(0, 1), 0.0) && TOLERANCE.is_close(l(0, 2), 0.0));
    MINI_CHECK(TOLERANCE.is_close(l(1, 2), 0.0));
}

MINI_TEST("Matrix", "Qr Decompose") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(3, 3, {12.0, -51.0, 4.0, 6.0, 167.0, -68.0, -4.0, 24.0, -41.0});
    auto [q, r] = a.qr_decompose();
    Matrix qt = q.transpose();
    Matrix qtq = qt.multiply(q);
    Matrix qr_prod = q.multiply(r);

    MINI_CHECK(TOLERANCE.is_close(qtq(0, 0), 1.0));
    MINI_CHECK(TOLERANCE.is_close(qtq(1, 1), 1.0));
    MINI_CHECK(TOLERANCE.is_close(qtq(2, 2), 1.0));
    MINI_CHECK(TOLERANCE.is_close(qtq(0, 1), 0.0) && TOLERANCE.is_close(qtq(0, 2), 0.0));
    MINI_CHECK(TOLERANCE.is_close(qr_prod(0, 0), 12.0));
    MINI_CHECK(TOLERANCE.is_close(qr_prod(1, 1), 167.0));
    MINI_CHECK(TOLERANCE.is_close(qr_prod(2, 2), -41.0));
}

MINI_TEST("Matrix", "Cholesky") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(3, 3, {4.0, 2.0, 2.0, 2.0, 5.0, 3.0, 2.0, 3.0, 6.0});
    auto l = a.cholesky();

    MINI_CHECK(l.has_value());
    Matrix lt = l->transpose();
    Matrix llt = l->multiply(lt);
    Matrix not_spd = Matrix::from_vec(2, 2, {1.0, 2.0, 2.0, 1.0});
    auto l_none = not_spd.cholesky();

    MINI_CHECK(TOLERANCE.is_close(llt(0, 0), 4.0) && TOLERANCE.is_close(llt(0, 1), 2.0));
    MINI_CHECK(TOLERANCE.is_close(llt(1, 0), 2.0) && TOLERANCE.is_close(llt(1, 1), 5.0));
    MINI_CHECK(TOLERANCE.is_close(llt(2, 2), 6.0));
    MINI_CHECK(!l_none.has_value());
}

MINI_TEST("Matrix", "Eigenvalues") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(3, 3, {3.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0});
    std::vector<double> evs = a.eigenvalues();
    std::sort(evs.begin(), evs.end());

    MINI_CHECK(evs.size() == 3);
    MINI_CHECK(TOLERANCE.is_close(evs[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(evs[1], 2.0));
    MINI_CHECK(TOLERANCE.is_close(evs[2], 3.0));
}

MINI_TEST("Matrix", "Svd") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(3, 3, {1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0});
    auto [u, sv, vt] = a.svd();
    std::sort(sv.begin(), sv.end(), std::greater<double>());

    MINI_CHECK(sv.size() == 3);
    MINI_CHECK(TOLERANCE.is_close(sv[0], 3.0));
    MINI_CHECK(TOLERANCE.is_close(sv[1], 2.0));
    MINI_CHECK(TOLERANCE.is_close(sv[2], 1.0));
}

MINI_TEST("Matrix", "Norms") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(2, 2, {1.0, -2.0, 3.0, -4.0});
    double nf = a.norm_frobenius();
    double n1 = a.norm_1();
    double ni = a.norm_inf();

    MINI_CHECK(TOLERANCE.is_close(nf, std::sqrt(30.0)));
    MINI_CHECK(TOLERANCE.is_close(n1, 6.0));
    MINI_CHECK(TOLERANCE.is_close(ni, 7.0));
}

MINI_TEST("Matrix", "Rank") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::identity(3);
    Matrix b = Matrix::from_vec(3, 3, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0});
    Matrix c = Matrix::zeros(3, 3);

    MINI_CHECK(a.rank() == 3);
    MINI_CHECK(b.rank() == 2);
    MINI_CHECK(c.rank() == 0);
}

MINI_TEST("Matrix", "Json Roundtrip") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    a.name = "test_matrix";
    a.json_dump("serialization/test_matrix.json");
    Matrix loaded = Matrix::json_load("serialization/test_matrix.json");

    MINI_CHECK(loaded.name == "test_matrix");
    MINI_CHECK(loaded.rows == 2 && loaded.cols == 3);
    MINI_CHECK(TOLERANCE.is_close(loaded(0, 0), 1.0) && TOLERANCE.is_close(loaded(1, 2), 6.0));
}

MINI_TEST("Matrix", "Protobuf Roundtrip") {
    // uncomment #include "matrix.h"
    Matrix a = Matrix::from_vec(2, 3, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    a.name = "test_matrix_proto";
    a.pb_dump("serialization/test_matrix.bin");
    Matrix loaded = Matrix::pb_load("serialization/test_matrix.bin");

    MINI_CHECK(loaded.name == "test_matrix_proto");
    MINI_CHECK(loaded.rows == 2 && loaded.cols == 3);
    MINI_CHECK(TOLERANCE.is_close(loaded(0, 0), 1.0) && TOLERANCE.is_close(loaded(1, 2), 6.0));
}

} // namespace session_cpp
