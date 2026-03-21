#include "session.h"
#include "mesh_iso.h"
#include "obb.h"

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

    // Row 0-8: SHELL — 9 TPMS types stacked in Y, one full period per 2×2×2 box.
    Obb box = Obb::from_points({Point(0, 0, 0), Point(2, 2, 2)});
    int    n    = 20;
    double step = 4.0;

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

    for (int i = 0; i < 9; ++i) {
        Mesh m = MeshIso::from_tpms(types[i].type, box, n, n, n, 0.0, 2.0, TpmsMode::SHELL, 0.1);
        add(session, std::move(m), types[i].name, 0, i * step);
    }

    // Row 9: SDF primitives + smooth booleans
    Obb sbox = Obb::from_points({Point(-2, -2, -2), Point(2, 2, 2)});
    int    sn    = 18;
    double sdf_y = 11.0 * step;

    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            return MeshIso::sdf_sphere(0, 0, 0, 1.2, x, y, z);
        }, sbox, sn, sn, sn),
        "sdf_sphere", 0 * step, sdf_y);

    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            return MeshIso::sdf_box(0, 0, 0, 1.0, 0.7, 1.3, x, y, z);
        }, sbox, sn, sn, sn),
        "sdf_box", 1 * step, sdf_y);

    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            return MeshIso::sdf_torus(0, 0, 0, 1.1, 0.4, x, y, z);
        }, sbox, sn, sn, sn),
        "sdf_torus", 2 * step, sdf_y);

    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            return MeshIso::sdf_capsule(Point(0, -1, 0), Point(0, 1, 0), 0.5, x, y, z);
        }, sbox, sn, sn, sn),
        "sdf_capsule", 3 * step, sdf_y);

    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            double a = MeshIso::sdf_sphere(-0.8, 0, 0, 0.8, x, y, z);
            double b = MeshIso::sdf_sphere( 0.8, 0, 0, 0.8, x, y, z);
            return MeshIso::smooth_union(a, b);
        }, sbox, sn, sn, sn),
        "smooth_union", 4 * step, sdf_y);

    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            double a = MeshIso::sdf_sphere(0, 0, 0, 1.2, x, y, z);
            double b = MeshIso::sdf_box(0, 0, 0, 0.8, 0.8, 0.8, x, y, z);
            return MeshIso::smooth_subtract(a, b);
        }, sbox, sn, sn, sn),
        "smooth_subtract", 5 * step, sdf_y);

    add(session,
        MeshIso::from_function([](double x, double y, double z) {
            double a = MeshIso::sdf_sphere(0, 0, 0, 1.4, x, y, z);
            double b = MeshIso::sdf_box(0, 0, 0, 1.1, 1.1, 1.1, x, y, z);
            return MeshIso::smooth_intersect(a, b);
        }, sbox, sn, sn, sn),
        "smooth_intersect", 6 * step, sdf_y);

    {
        Obb gbox = Obb::from_points({Point(-1.6, -1.6, -1.6), Point(1.6, 1.6, 1.6)});
        add(session,
            MeshIso::from_function([](double x, double y, double z) {
                double tpms  = MeshIso::eval(TpmsType::GYROID, x, y, z, 1.0);
                double shell = std::abs(MeshIso::sdf_sphere(0, 0, 0, 1.3, x, y, z)) - 0.08;
                return MeshIso::smooth_union(tpms * 0.3, shell);
            }, gbox, sn, sn, sn),
            "gyroid_sphere_shell", 7 * step, sdf_y);
    }

    session.pb_dump(base + "/isomesh_demo.pb");
    std::cout << "Saved to session_data/isomesh_demo.pb\n";
    return 0;
}
