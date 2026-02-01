#include "xform.h"
#include "point.h"
#include "plane.h"

#include "xform.pb.h"

namespace session_cpp {

Xform::Xform() {
    m = {0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0};
    m[0] = 1.0;
    m[5] = 1.0;
    m[10] = 1.0;
    m[15] = 1.0;
}

Xform::Xform(const std::array<double, 16>& matrix) : m(matrix) {}

/// Copy constructor (creates a new guid while copying data)
Xform::Xform(const Xform& other)
    : guid(::guid()),
      name(other.name),
      m(other.m) {}

/// Copy assignment (creates a new guid while copying data)
Xform& Xform::operator=(const Xform& other) {
    if (this != &other) {
        guid = ::guid();
        name = other.name;
        m = other.m;
    }
    return *this;
}

Xform Xform::identity() {
    return Xform();
}

Xform Xform::from_matrix(const std::array<double, 16>& matrix) {
    return Xform(matrix);
}

Xform Xform::translation(double x, double y, double z) {
    Xform xform;
    xform.m[12] = x;
    xform.m[13] = y;
    xform.m[14] = z;
    return xform;
}

Xform Xform::scaling(double x, double y, double z) {
    Xform xform;
    xform.m[0] = x;
    xform.m[5] = y;
    xform.m[10] = z;
    return xform;
}

Xform Xform::rotation_x(double angle_radians) {
    Xform xform;
    double cos_angle = std::cos(angle_radians);
    double sin_angle = std::sin(angle_radians);
    xform.m[5] = cos_angle;
    xform.m[6] = sin_angle;
    xform.m[9] = -sin_angle;
    xform.m[10] = cos_angle;
    return xform;
}

Xform Xform::rotation_y(double angle_radians) {
    Xform xform;
    double cos_angle = std::cos(angle_radians);
    double sin_angle = std::sin(angle_radians);
    xform.m[0] = cos_angle;
    xform.m[2] = -sin_angle;
    xform.m[8] = sin_angle;
    xform.m[10] = cos_angle;
    return xform;
}

Xform Xform::rotation_z(double angle_radians) {
    Xform xform;
    double cos_angle = std::cos(angle_radians);
    double sin_angle = std::sin(angle_radians);
    xform.m[0] = cos_angle;
    xform.m[1] = sin_angle;
    xform.m[4] = -sin_angle;
    xform.m[5] = cos_angle;
    return xform;
}

Xform Xform::rotation(Vector& axis, double angle_radians) {
    Xform xform;
    axis.normalize_self();
    
    double cos_angle = std::cos(angle_radians);
    double sin_angle = std::sin(angle_radians);
    double one_minus_cos = 1.0 - cos_angle;

    double xx = axis[0] * axis[0];
    double xy = axis[0] * axis[1];
    double xz = axis[0] * axis[2];
    double yy = axis[1] * axis[1];
    double yz = axis[1] * axis[2];
    double zz = axis[2] * axis[2];

    xform.m[0] = cos_angle + xx * one_minus_cos;
    xform.m[1] = xy * one_minus_cos + axis[2] * sin_angle;
    xform.m[2] = xz * one_minus_cos - axis[1] * sin_angle;

    xform.m[4] = xy * one_minus_cos - axis[2] * sin_angle;
    xform.m[5] = cos_angle + yy * one_minus_cos;
    xform.m[6] = yz * one_minus_cos + axis[0] * sin_angle;

    xform.m[8] = xz * one_minus_cos + axis[1] * sin_angle;
    xform.m[9] = yz * one_minus_cos - axis[0] * sin_angle;
    xform.m[10] = cos_angle + zz * one_minus_cos;

    return xform;
}

Xform Xform::change_basis(Point& origin_1, Vector& x_axis_1, Vector& y_axis_1, Vector& z_axis_1,
                           Point& origin_0, Vector& x_axis_0, Vector& y_axis_0, Vector& z_axis_0) {
    double a = x_axis_1.dot(y_axis_1);
    double b = x_axis_1.dot(z_axis_1);
    double c = y_axis_1.dot(z_axis_1);

    double r[3][6] = {
        {x_axis_1.dot(x_axis_1), a, b, x_axis_1.dot(x_axis_0), x_axis_1.dot(y_axis_0), x_axis_1.dot(z_axis_0)},
        {a, y_axis_1.dot(y_axis_1), c, y_axis_1.dot(x_axis_0), y_axis_1.dot(y_axis_0), y_axis_1.dot(z_axis_0)},
        {b, c, z_axis_1.dot(z_axis_1), z_axis_1.dot(x_axis_0), z_axis_1.dot(y_axis_0), z_axis_1.dot(z_axis_0)}
    };

    int i0 = (r[0][0] >= r[1][1]) ? 0 : 1;
    if (r[2][2] > r[i0][i0]) i0 = 2;
    int i1 = (i0 + 1) % 3;
    int i2 = (i1 + 1) % 3;

    if (r[i0][i0] == 0.0) return Xform::identity();

    double d = 1.0 / r[i0][i0];
    for (int j = 0; j < 6; j++) r[i0][j] *= d;
    r[i0][i0] = 1.0;

    if (r[i1][i0] != 0.0) {
        d = -r[i1][i0];
        for (int j = 0; j < 6; j++) r[i1][j] += d * r[i0][j];
        r[i1][i0] = 0.0;
    }
    if (r[i2][i0] != 0.0) {
        d = -r[i2][i0];
        for (int j = 0; j < 6; j++) r[i2][j] += d * r[i0][j];
        r[i2][i0] = 0.0;
    }

    if (std::abs(r[i1][i1]) < std::abs(r[i2][i2])) {
        std::swap(i1, i2);
    }
    if (r[i1][i1] == 0.0) return Xform::identity();

    d = 1.0 / r[i1][i1];
    for (int j = 0; j < 6; j++) r[i1][j] *= d;
    r[i1][i1] = 1.0;

    if (r[i0][i1] != 0.0) {
        d = -r[i0][i1];
        for (int j = 0; j < 6; j++) r[i0][j] += d * r[i1][j];
        r[i0][i1] = 0.0;
    }
    if (r[i2][i1] != 0.0) {
        d = -r[i2][i1];
        for (int j = 0; j < 6; j++) r[i2][j] += d * r[i1][j];
        r[i2][i1] = 0.0;
    }

    if (r[i2][i2] == 0.0) return Xform::identity();

    d = 1.0 / r[i2][i2];
    for (int j = 0; j < 6; j++) r[i2][j] *= d;
    r[i2][i2] = 1.0;

    if (r[i0][i2] != 0.0) {
        d = -r[i0][i2];
        for (int j = 0; j < 6; j++) r[i0][j] += d * r[i2][j];
        r[i0][i2] = 0.0;
    }
    if (r[i1][i2] != 0.0) {
        d = -r[i1][i2];
        for (int j = 0; j < 6; j++) r[i1][j] += d * r[i2][j];
        r[i1][i2] = 0.0;
    }

    Xform m_xform;
    m_xform.m[0] = static_cast<double>(r[0][3]);
    m_xform.m[4] = static_cast<double>(r[0][4]);
    m_xform.m[8] = static_cast<double>(r[0][5]);
    m_xform.m[1] = static_cast<double>(r[1][3]);
    m_xform.m[5] = static_cast<double>(r[1][4]);
    m_xform.m[9] = static_cast<double>(r[1][5]);
    m_xform.m[2] = static_cast<double>(r[2][3]);
    m_xform.m[6] = static_cast<double>(r[2][4]);
    m_xform.m[10] = static_cast<double>(r[2][5]);

    Xform t0 = translation(-origin_1[0], -origin_1[1], -origin_1[2]);
    Xform t2 = translation(origin_0[0], origin_0[1], origin_0[2]);
    return t2 * (m_xform * t0);
}

Xform Xform::plane_to_plane(const Plane& plane_from, const Plane& plane_to) {
    Vector x0 = plane_from.x_axis(), y0 = plane_from.y_axis(), z0 = plane_from.z_axis();
    Vector x1 = plane_to.x_axis(), y1 = plane_to.y_axis(), z1 = plane_to.z_axis();
    x0.normalize_self(); y0.normalize_self(); z0.normalize_self();
    x1.normalize_self(); y1.normalize_self(); z1.normalize_self();

    const Point& origin_0 = plane_from.origin();
    const Point& origin_1 = plane_to.origin();

    Xform t0 = translation(-origin_0[0], -origin_0[1], -origin_0[2]);

    Xform f0;
    f0.m[0] = x0[0]; f0.m[1] = x0[1]; f0.m[2] = x0[2];
    f0.m[4] = y0[0]; f0.m[5] = y0[1]; f0.m[6] = y0[2];
    f0.m[8] = z0[0]; f0.m[9] = z0[1]; f0.m[10] = z0[2];

    Xform f1;
    f1.m[0] = x1[0]; f1.m[4] = x1[1]; f1.m[8] = x1[2];
    f1.m[1] = y1[0]; f1.m[5] = y1[1]; f1.m[9] = y1[2];
    f1.m[2] = z1[0]; f1.m[6] = z1[1]; f1.m[10] = z1[2];

    Xform r = f1 * f0;
    Xform t1 = translation(origin_1[0], origin_1[1], origin_1[2]);
    return t1 * (r * t0);
}

Xform Xform::plane_to_xy(Point& origin, Vector& x_axis, Vector& y_axis, Vector& z_axis) {
    Vector x = x_axis, y = y_axis, z = z_axis;
    x.normalize_self(); y.normalize_self(); z.normalize_self();

    Xform t = translation(-origin[0], -origin[1], -origin[2]);
    Xform f;
    f.m[0] = x[0]; f.m[1] = x[1]; f.m[2] = x[2];
    f.m[4] = y[0]; f.m[5] = y[1]; f.m[6] = y[2];
    f.m[8] = z[0]; f.m[9] = z[1]; f.m[10] = z[2];
    return f * t;
}

Xform Xform::xy_to_plane(Point& origin, Vector& x_axis, Vector& y_axis, Vector& z_axis) {
    Vector x = x_axis, y = y_axis, z = z_axis;
    x.normalize_self(); y.normalize_self(); z.normalize_self();

    Xform f;
    f.m[0] = x[0]; f.m[4] = y[0]; f.m[8] = z[0];
    f.m[1] = x[1]; f.m[5] = y[1]; f.m[9] = z[1];
    f.m[2] = x[2]; f.m[6] = y[2]; f.m[10] = z[2];

    Xform t = translation(origin[0], origin[1], origin[2]);
    return t * f;
}

Xform Xform::to_frame(const Plane& frame) {
    // Transform from world XY to target frame (same as COMPAS from_frame)
    // Matrix columns are: xaxis, yaxis, zaxis, origin
    Vector x = frame.x_axis(), y = frame.y_axis(), z = frame.z_axis();
    x.normalize_self(); y.normalize_self(); z.normalize_self();
    const Point& o = frame.origin();

    Xform xf;
    xf.m[0] = x[0]; xf.m[4] = y[0]; xf.m[8]  = z[0]; xf.m[12] = o[0];
    xf.m[1] = x[1]; xf.m[5] = y[1]; xf.m[9]  = z[1]; xf.m[13] = o[1];
    xf.m[2] = x[2]; xf.m[6] = y[2]; xf.m[10] = z[2]; xf.m[14] = o[2];
    xf.m[3] = 0.0;  xf.m[7] = 0.0;  xf.m[11] = 0.0;  xf.m[15] = 1.0;
    return xf;
}

Xform Xform::scale_xyz(double scale_x, double scale_y, double scale_z) {
    Xform xform;
    xform.m[0] = scale_x;
    xform.m[5] = scale_y;
    xform.m[10] = scale_z;
    return xform;
}

Xform Xform::scale_uniform(Point& origin, double scale_value) {
    Xform t0 = translation(-origin[0], -origin[1], -origin[2]);
    Xform t1 = scaling(scale_value, scale_value, scale_value);
    Xform t2 = translation(origin[0], origin[1], origin[2]);
    return t2 * (t1 * t0);
}

Xform Xform::scale_non_uniform(Point& origin, double scale_x, double scale_y, double scale_z) {
    Xform t0 = translation(-origin[0], -origin[1], -origin[2]);
    Xform t1 = scale_xyz(scale_x, scale_y, scale_z);
    Xform t2 = translation(origin[0], origin[1], origin[2]);
    return t2 * (t1 * t0);
}

Xform Xform::axis_rotation(double angle, Vector& axis) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    double ux = axis[0];
    double uy = axis[1];
    double uz = axis[2];
    double t = 1.0 - c;

    Xform xform;
    xform.m[0] = t * ux * ux + c;
    xform.m[4] = t * ux * uy - uz * s;
    xform.m[8] = t * ux * uz + uy * s;

    xform.m[1] = t * ux * uy + uz * s;
    xform.m[5] = t * uy * uy + c;
    xform.m[9] = t * uy * uz - ux * s;

    xform.m[2] = t * ux * uz - uy * s;
    xform.m[6] = t * uy * uz + ux * s;
    xform.m[10] = t * uz * uz + c;

    return xform;
}

Xform Xform::look_at_rh(const Point& eye, const Point& target, const Vector& up) {
    Vector f = target - eye;
    f.normalize_self();
    Vector up_copy = up;
    up_copy.normalize_self();
    Vector s = f.cross(up_copy);
    s.normalize_self();
    Vector u = s.cross(f);
    
    Xform xform;
    xform.m[0] = s[0];
    xform.m[4] = s[1];
    xform.m[8] = s[2];
    
    xform.m[1] = u[0];
    xform.m[5] = u[1];
    xform.m[9] = u[2];
    
    xform.m[2] = -f[0];
    xform.m[6] = -f[1];
    xform.m[10] = -f[2];
    
    Vector eye_vec(eye[0], eye[1], eye[2]);
    xform.m[12] = -s.dot(eye_vec);
    xform.m[13] = -u.dot(eye_vec);
    xform.m[14] = f.dot(eye_vec);
    
    return xform;
}

std::optional<Xform> Xform::inverse() const {
    double a00 = m[0], a01 = m[4], a02 = m[8];
    double a10 = m[1], a11 = m[5], a12 = m[9];
    double a20 = m[2], a21 = m[6], a22 = m[10];

    double det = a00 * (a11 * a22 - a12 * a21) 
              - a01 * (a10 * a22 - a12 * a20)
              + a02 * (a10 * a21 - a11 * a20);
    
    if (std::abs(det) < 1e-12) {
        return std::nullopt;
    }

    double inv_det = 1.0 / det;

    double m00 = (a11 * a22 - a12 * a21) * inv_det;
    double m01 = (a02 * a21 - a01 * a22) * inv_det;
    double m02 = (a01 * a12 - a02 * a11) * inv_det;
    double m10 = (a12 * a20 - a10 * a22) * inv_det;
    double m11 = (a00 * a22 - a02 * a20) * inv_det;
    double m12 = (a02 * a10 - a00 * a12) * inv_det;
    double m20 = (a10 * a21 - a11 * a20) * inv_det;
    double m21 = (a01 * a20 - a00 * a21) * inv_det;
    double m22 = (a00 * a11 - a01 * a10) * inv_det;

    double tx = m[12], ty = m[13], tz = m[14];
    double itx = -(m00 * tx + m01 * ty + m02 * tz);
    double ity = -(m10 * tx + m11 * ty + m12 * tz);
    double itz = -(m20 * tx + m21 * ty + m22 * tz);

    Xform res;
    res.guid = "";
    res.name = "";
    res.m[0] = m00;  res.m[4] = m01;  res.m[8] = m02;   res.m[12] = itx;
    res.m[1] = m10;  res.m[5] = m11;  res.m[9] = m12;   res.m[13] = ity;
    res.m[2] = m20;  res.m[6] = m21;  res.m[10] = m22;  res.m[14] = itz;
    
    return res;
}

bool Xform::is_identity() const {
    Xform identity;
    for (int i = 0; i < 16; i++) {
        if (std::abs(m[i] - identity.m[i]) > 1e-10) {
            return false;
        }
    }
    return true;
}

Point Xform::transformed_point(const Point& point) const {
    double x = point[0];
    double y = point[1];
    double z = point[2];
    double w = m[3] * x + m[7] * y + m[11] * z + m[15];
    double w_inv = (std::abs(w) > 1e-10) ? 1.0 / w : 1.0;

    return Point(
        (m[0] * x + m[4] * y + m[8] * z + m[12]) * w_inv,
        (m[1] * x + m[5] * y + m[9] * z + m[13]) * w_inv,
        (m[2] * x + m[6] * y + m[10] * z + m[14]) * w_inv
    );
}

Vector Xform::transformed_vector(const Vector& vector) const {
    double x = vector[0];
    double y = vector[1];
    double z = vector[2];

    return Vector(
        m[0] * x + m[4] * y + m[8] * z,
        m[1] * x + m[5] * y + m[9] * z,
        m[2] * x + m[6] * y + m[10] * z
    );
}

void Xform::transform_point(Point& point) const {
    double x = point[0];
    double y = point[1];
    double z = point[2];
    double w = m[3] * x + m[7] * y + m[11] * z + m[15];
    double w_inv = (std::abs(w) > 1e-10) ? 1.0 / w : 1.0;

    point[0] = (m[0] * x + m[4] * y + m[8] * z + m[12]) * w_inv;
    point[1] = (m[1] * x + m[5] * y + m[9] * z + m[13]) * w_inv;
    point[2] = (m[2] * x + m[6] * y + m[10] * z + m[14]) * w_inv;
}

void Xform::transform_vector(Vector& vector) const {
    double x = vector[0];
    double y = vector[1];
    double z = vector[2];

    vector[0] = m[0] * x + m[4] * y + m[8] * z;
    vector[1] = m[1] * x + m[5] * y + m[9] * z;
    vector[2] = m[2] * x + m[6] * y + m[10] * z;
}

nlohmann::ordered_json Xform::jsondump() const {
    // Alphabetical order to match Rust's serde_json
    nlohmann::ordered_json data;
    data["guid"] = guid;
    data["m"] = m;
    data["name"] = name;
    data["type"] = "Xform";
    return data;
}

Xform Xform::jsonload(const nlohmann::json& data) {
    Xform xform;
    xform.guid = data["guid"].get<std::string>();
    xform.name = data["name"].get<std::string>();
    xform.m = data["m"].get<std::array<double, 16>>();
    return xform;
}

std::string Xform::json_dumps() const {
    return jsondump().dump();
}

Xform Xform::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Xform::json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

Xform Xform::json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data = nlohmann::json::parse(file);
    return jsonload(data);
}

std::string Xform::pb_dumps() const {
    session_proto::Xform proto;
    proto.set_guid(guid);
    proto.set_name(name);
    for (int i = 0; i < 16; ++i) {
        proto.add_matrix(m[i]);
    }
    return proto.SerializeAsString();
}

Xform Xform::pb_loads(const std::string& data) {
    session_proto::Xform proto;
    proto.ParseFromString(data);

    Xform xform;
    xform.guid = proto.guid();
    xform.name = proto.name();
    for (int i = 0; i < 16 && i < proto.matrix_size(); ++i) {
        xform.m[i] = proto.matrix(i);
    }
    return xform;
}

void Xform::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

Xform Xform::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
    return pb_loads(data);
}

Xform Xform::operator*(const Xform& other) const {
    Xform result;
    result.m = {0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double sum = 0.0;
            for (int k = 0; k < 4; k++) {
                sum += m[k * 4 + i] * other.m[j * 4 + k];
            }
            result.m[j * 4 + i] = sum;
        }
    }

    return result;
}

Xform& Xform::operator*=(const Xform& other) {
    *this = *this * other;
    return *this;
}

double& Xform::operator()(int row, int col) {
    if (row < 0 || row >= 4 || col < 0 || col >= 4) {
        throw std::out_of_range("Index out of bounds: (" + std::to_string(row) + ", " + std::to_string(col) + ")");
    }
    return m[col * 4 + row];
}

const double& Xform::operator()(int row, int col) const {
    if (row < 0 || row >= 4 || col < 0 || col >= 4) {
        throw std::out_of_range("Index out of bounds: (" + std::to_string(row) + ", " + std::to_string(col) + ")");
    }
    return m[col * 4 + row];
}

bool Xform::operator==(const Xform& other) const {
    constexpr double tolerance = 1e-10;
    for (size_t i = 0; i < 16; ++i) {
        if (std::abs(m[i] - other.m[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

bool Xform::operator!=(const Xform& other) const {
    return !(*this == other);
}

std::string Xform::str() const {
    std::ostringstream oss;
    for (int i = 0; i < 4; i++) {
        oss << "[" << fmt::format("{:.6f}", m[i]) << ", "
            << fmt::format("{:.6f}", m[4 + i]) << ", "
            << fmt::format("{:.6f}", m[8 + i]) << ", "
            << fmt::format("{:.6f}", m[12 + i]) << "]";
        if (i < 3) oss << "\n";
    }
    return oss.str();
}

std::string Xform::repr() const {
    return fmt::format("Xform({}, {})", name, guid.substr(0, 8));
}

} // namespace session_cpp
