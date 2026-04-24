// ─── ss_e_op_4 ──────────────────────────────────────────────────────────────
// Verbatim port of wood_joint_lib.cpp:2006-2191. Parametric finger joint
// with N tenons (controlled by joint.divisions). Takes 9 extra params;
// for the default dispatcher call we use the values from ss_e_op_5's usage
// at wood line 2210.
static void ss_e_op_4(WoodJoint& joint,
                       double t = 0.0, bool chamfer = false, bool female_modify_outline = true,
                       double x0 = -0.5, double x1 = 0.5,
                       double y0 = -0.5, double y1 = 0.5,
                       double z_ext0 = -0.5, double z_ext1 = 0.5) {
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // wood::joint parameters
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int number_of_tenons = joint.divisions;
    std::array<double, 2> x = { x0, x1 };
    std::array<double, 2> y = { y0, y1 };
    std::array<double, 2> z_ext = { z_ext0, z_ext1 };

    number_of_tenons = std::min(50, std::max(2, number_of_tenons)) * 2;
    double step = 1.0 / ((double)number_of_tenons - 1);
    std::array<double, 2> z = { z_ext[0] + step, z_ext[1] - step };
    number_of_tenons -= 2;
    step = 1.0 / ((double)number_of_tenons - 1);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Male
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    for (int j = 0; j < 2; j++) {
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // memory and variables
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        joint.m_outlines[j].resize(2);
        int sign = j == 0 ? -1 : 1;
        std::vector<Point> pts;
        pts.reserve(4 + number_of_tenons * 2);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // tenon interpolation
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        pts.emplace_back(sign * x[1], y[j], z_ext[1]);
        pts.emplace_back(x[1] + j * 0.01, y[j], z_ext[1]);

        if (joint.divisions > 0) {
            for (int i = 0; i < number_of_tenons; i++) {
                double z_ = z[1] + (z[0] - z[1]) * step * i;
                double z_mid = i % 2 == 0
                    ? z_ + (z[0] - z[1]) * step * t * 0.05
                    : z_ - (z[0] - z[1]) * step * t * 0.05;
                double z_chamfer_val = i % 2 == 0
                    ? z_ + (z[0] - z[1]) * step * ((t * 0.5) + ((1 - t) * 0.5 * 0.25))
                    : z_ - (z[0] - z[1]) * step * ((t * 0.5) + ((1 - t) * 0.5 * 0.25));
                z_ = i % 2 == 0
                    ? z_ + (z[0] - z[1]) * step * t * 0.5
                    : z_ - (z[0] - z[1]) * step * t * 0.5;

                if (i % 2 == 0) {
                    if (!female_modify_outline) pts.emplace_back(x[1], y[j], z_mid);
                    pts.emplace_back(x[1], y[j], z_);
                    pts.emplace_back(x[0], y[j], z_);
                    if (chamfer) pts.emplace_back(-0.75 + x[0], y[j], z_chamfer_val);
                } else {
                    if (chamfer) pts.emplace_back(-0.75 + x[0], y[j], z_chamfer_val);
                    pts.emplace_back(x[0], y[j], z_);
                    pts.emplace_back(x[1], y[j], z_);
                    if (!female_modify_outline) pts.emplace_back(x[1], y[j], z_mid);
                }
            }
        }

        pts.emplace_back(x[1] + j * 0.01, y[j], z_ext[0]);
        pts.emplace_back(sign * x[1], y[j], z_ext[0]);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // cut outlines
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        joint.m_outlines[j][0] = Polyline(pts);
        joint.m_outlines[j][1] = Polyline({Point(sign * x[1], y[j], z_ext[1]), Point(sign * x[1], y[j], z_ext[0])});
    }

    // Female outlines
    int fmo_count = 2 * (int)female_modify_outline;
    for (int j = 0; j < 2; j++) {
        if (joint.divisions == 0)
            joint.f_outlines[j].resize(fmo_count);
        else
            joint.f_outlines[j].resize(fmo_count + number_of_tenons);
        int sign = j == 0 ? 1 : -1;
        int j_inv = j == 0 ? 1 : 0;

        if (female_modify_outline) {
            joint.f_outlines[j][0] = Polyline({
                Point(y[j_inv], sign * y[1], z_ext[1]),
                Point(y[j_inv], 3 * y[0], z_ext[1]),
                Point(y[j_inv], 3 * y[0], z_ext[0]),
                Point(y[j_inv], sign * y[1], z_ext[0]),
            });
            joint.f_outlines[j][1] = Polyline({
                Point(y[j_inv], sign * y[1], z_ext[1]),
                Point(y[j_inv], sign * y[1], z_ext[1]),
            });
        }

        if (joint.divisions > 0) {
            for (int i = 0; i < number_of_tenons; i += 2) {
                double z_ = z[1] + (z[0] - z[1]) * step * i;
                z_ = i % 2 == 0
                    ? z_ + (z[0] - z[1]) * step * t * 0.5
                    : z_ - (z[0] - z[1]) * step * t * 0.5;
                std::vector<Point> hole_pts;
                hole_pts.reserve(5);
                hole_pts.emplace_back(y[j_inv], y[0], z_);
                hole_pts.emplace_back(y[j_inv], y[1], z_);

                double z2 = z[1] + (z[0] - z[1]) * step * (i + 1);
                z2 = (i + 1) % 2 == 0
                    ? z2 + (z[0] - z[1]) * step * t * 0.5
                    : z2 - (z[0] - z[1]) * step * t * 0.5;
                hole_pts.emplace_back(y[j_inv], y[1], z2);
                hole_pts.emplace_back(y[j_inv], y[0], z2);
                hole_pts.push_back(hole_pts.front());

                joint.f_outlines[j][fmo_count + i] = Polyline(hole_pts);
                joint.f_outlines[j][fmo_count + i + 1] = Polyline(hole_pts);
            }
        }
    }

    // Cut types
    joint.m_cut_types[0] = { wood_cut::insert_between_multiple_edges, wood_cut::insert_between_multiple_edges };
    joint.m_cut_types[1] = { wood_cut::insert_between_multiple_edges, wood_cut::insert_between_multiple_edges };
    for (int j = 0; j < 2; j++) {
        std::vector<int> fct;
        if (female_modify_outline) {
            fct.push_back(wood_cut::insert_between_multiple_edges);
            fct.push_back(wood_cut::insert_between_multiple_edges);
        }
        if (joint.divisions > 0) {
            for (int i = 0; i < number_of_tenons; i += 2) {
                fct.push_back(wood_cut::hole);
                fct.push_back(wood_cut::hole);
            }
        }
        joint.f_cut_types[j] = fct;
    }
}
