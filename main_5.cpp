#include "session.h"
#include "mesh_iso.h"
#include "boundingbox.h"

using namespace session_cpp;

static void add(Session& session, Mesh m, const std::string& name, double tx, double ty) {
    m.name = name;
    m.xform = Xform::translation(tx, ty, 0);
    m.transform();
    std::cout << name << ": " << m.number_of_vertices() << "v " << m.number_of_faces() << "f\n";
    session.add_mesh(std::make_shared<Mesh>(std::move(m)));
}

int main() {
    Session session("isomesh_demo");
    std::string base = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data").string();

    BoundingBox box = BoundingBox::from_points({Point(0, 0, 0), Point(2, 2, 2)});
    int n = 20;
    double period = 1.0;
    double step = 12.0;

    struct { TpmsType type; const char* name; } types[] = {
        {TpmsType::GYROID,         "gyroid"},
        {TpmsType::SCHWARZ_P,      "schwarz_p"},
        {TpmsType::DIAMOND,        "diamond"},
        {TpmsType::NEOVIUS,        "neovius"},
        {TpmsType::IWP,            "iwp"},
        {TpmsType::LIDINOID,       "lidinoid"},
        {TpmsType::FISCHER_KOCH_S, "fischer_koch_s"},
        {TpmsType::FRD,            "frd"},
        {TpmsType::PMY,            "pmy"},
    };

    // Row 0: SOLID
    for (int i = 0; i < 9; ++i) {
        Mesh m = MeshIso::from_tpms(types[i].type, box, n, n, n, 0.0, period, TpmsMode::SOLID);
        add(session, std::move(m), std::string(types[i].name) + "_solid", i * step, 0);
    }

    // Row 1: SHEET
    for (int i = 0; i < 9; ++i) {
        Mesh m = MeshIso::from_tpms(types[i].type, box, n, n, n, 0.0, period, TpmsMode::SHEET, 0.15);
        add(session, std::move(m), std::string(types[i].name) + "_sheet", i * step, step);
    }

    // Row 2: SHELL
    for (int i = 0; i < 9; ++i) {
        Mesh m = MeshIso::from_tpms(types[i].type, box, n, n, n, 0.0, period, TpmsMode::SHELL, 0.1);
        add(session, std::move(m), std::string(types[i].name) + "_shell", i * step, 2 * step);
    }

    // Row 3: SDF primitives + smooth booleans
    BoundingBox sbox = BoundingBox::from_points({Point(-2, -2, -2), Point(2, 2, 2)});
    int sn = 30;

    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            return MeshIso::sdf_sphere(0, 0, 0, 1.2, x, y, z);
        }, sbox, sn, sn, sn),
        "sdf_sphere", 0, 3 * step);

    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            return MeshIso::sdf_box(0, 0, 0, 1.0, 0.7, 1.3, x, y, z);
        }, sbox, sn, sn, sn),
        "sdf_box", step, 3 * step);

    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            return MeshIso::sdf_torus(0, 0, 0, 1.1, 0.4, x, y, z);
        }, sbox, sn, sn, sn),
        "sdf_torus", 2 * step, 3 * step);

    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            return MeshIso::sdf_capsule(Point(0, -1, 0), Point(0, 1, 0), 0.5, x, y, z);
        }, sbox, sn, sn, sn),
        "sdf_capsule", 3 * step, 3 * step);

    // Smooth union: two offset spheres
    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            double a = MeshIso::sdf_sphere(-0.8, 0, 0, 0.8, x, y, z);
            double b = MeshIso::sdf_sphere( 0.8, 0, 0, 0.8, x, y, z);
            return MeshIso::smooth_union(a, b, 4.0);
        }, sbox, sn, sn, sn),
        "smooth_union", 4 * step, 3 * step);

    // Smooth subtract: sphere minus box
    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            double a = MeshIso::sdf_sphere(0, 0, 0, 1.2, x, y, z);
            double b = MeshIso::sdf_box(0, 0, 0, 0.8, 0.8, 0.8, x, y, z);
            return MeshIso::smooth_subtract(a, b, 6.0);
        }, sbox, sn, sn, sn),
        "smooth_subtract", 5 * step, 3 * step);

    // Smooth intersect: sphere and box
    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            double a = MeshIso::sdf_sphere(0, 0, 0, 1.4, x, y, z);
            double b = MeshIso::sdf_box(0, 0, 0, 1.1, 1.1, 1.1, x, y, z);
            return MeshIso::smooth_intersect(a, b, 6.0);
        }, sbox, sn, sn, sn),
        "smooth_intersect", 6 * step, 3 * step);

    // Gyroid inside sphere shell (TPMS clipped to spherical domain)
    {
        BoundingBox gbox = BoundingBox::from_points({Point(-1.6, -1.6, -1.6), Point(1.6, 1.6, 1.6)});
        add(session,
            MeshIso::from_function([](double x, double y, double z) {
                double tpms = MeshIso::eval(TpmsType::GYROID, x, y, z, 1.0);
                double shell = std::abs(MeshIso::sdf_sphere(0, 0, 0, 1.3, x, y, z)) - 0.08;
                return MeshIso::smooth_union(tpms * 0.3, shell, 3.0);
            }, gbox, sn, sn, sn),
            "gyroid_sphere_shell", 7 * step, 3 * step);
    }

    session.pb_dump(base + "/isomesh_demo.pb");
    std::cout << "Saved to session_data/isomesh_demo.pb\n";
    return 0;
}
