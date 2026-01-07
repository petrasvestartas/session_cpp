#include "step_writer.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <set>
#include <regex>

namespace session_cpp {

std::string StepWriter::format_double(double v) {
    if (std::abs(v) < 1e-15) v = 0.0;
    std::ostringstream oss;
    oss << std::setprecision(15) << v;
    std::string s = oss.str();
    if (s.find('.') == std::string::npos && s.find('E') == std::string::npos) {
        s += ".";
    }
    return s;
}

std::string StepWriter::format_point(double x, double y, double z) {
    return "(" + format_double(x) + "," + format_double(y) + "," + format_double(z) + ")";
}

void StepWriter::write(const std::string& filepath, const StepFile& step) {
    std::ofstream out(filepath);
    if (!out.is_open()) {
        throw std::runtime_error("Cannot write STEP file: " + filepath);
    }

    // Find max entity ID from existing entities
    int next_id = 1;
    for (const auto& entity : step.entities) {
        if (entity.id >= next_id) next_id = entity.id + 1;
    }

    // Collect IDs of curves/surfaces already in raw entities
    std::set<int> existing_curve_ids, existing_surface_ids;
    for (const auto& entity : step.entities) {
        if (entity.data.find("B_SPLINE_CURVE_WITH_KNOTS") != std::string::npos) {
            existing_curve_ids.insert(entity.id);
        }
        if (entity.data.find("B_SPLINE_SURFACE_WITH_KNOTS") != std::string::npos) {
            existing_surface_ids.insert(entity.id);
        }
    }

    // Pre-calculate IDs for new curves and surfaces
    std::vector<int> new_curve_ids;
    std::vector<int> new_surface_ids;
    int temp_id = next_id;
    for (const auto& [id, curve] : step.curves) {
        if (existing_curve_ids.count(id)) continue;
        temp_id += curve.cv_count();  // control points
        new_curve_ids.push_back(temp_id++);  // curve entity
    }
    for (const auto& [id, surf] : step.surfaces) {
        if (existing_surface_ids.count(id)) continue;
        temp_id += surf.cv_count(0) * surf.cv_count(1);  // control points
        new_surface_ids.push_back(temp_id++);  // surface entity
    }

    // Write header
    out << "ISO-10303-21;\n";
    out << "HEADER;\n";
    out << "FILE_DESCRIPTION((''), '2;1');\n";
    out << "FILE_NAME('" << step.filename << "', '" << step.timestamp << "', (''), (''), '', '"
        << step.originating_system << "', '');\n";
    out << "FILE_SCHEMA(('" << (step.schema.empty() ? "AUTOMOTIVE_DESIGN" : step.schema) << "'));\n";
    out << "ENDSEC;\n";
    out << "DATA;\n";

    // Write existing entities, updating CARTESIAN_POINT and GEOMETRIC_CURVE_SET
    for (const auto& entity : step.entities) {
        out << "#" << entity.id << "=";

        if (entity.type == "CARTESIAN_POINT") {
            auto it = step.points.find(entity.id);
            if (it != step.points.end()) {
                const Point& pt = it->second;
                std::string name;
                size_t q1 = entity.data.find('\'');
                size_t q2 = entity.data.find('\'', q1 + 1);
                if (q1 != std::string::npos && q2 != std::string::npos) {
                    name = entity.data.substr(q1 + 1, q2 - q1 - 1);
                }
                out << "CARTESIAN_POINT('" << name << "'," << format_point(pt[0], pt[1], pt[2]) << ")";
            } else {
                out << entity.data;
            }
        } else if (entity.type == "GEOMETRIC_CURVE_SET" && !new_curve_ids.empty()) {
            // Append new curve references to GEOMETRIC_CURVE_SET
            // Format: GEOMETRIC_CURVE_SET('name',(curve_refs...))
            // Insert before the second-to-last ) (the one closing the curve list)
            std::string data = entity.data;
            size_t last_paren = data.rfind(')');
            size_t second_last = data.rfind(')', last_paren - 1);
            if (second_last != std::string::npos) {
                std::string new_refs;
                for (int cid : new_curve_ids) {
                    new_refs += ",#" + std::to_string(cid);
                }
                data.insert(second_last, new_refs);
            }
            out << data;
        } else if (entity.type == "SHELL_BASED_SURFACE_MODEL" && !new_surface_ids.empty()) {
            // Append new surface references
            std::string data = entity.data;
            size_t close_paren = data.rfind(')');
            if (close_paren != std::string::npos && close_paren > 0) {
                std::string new_refs;
                for (int sid : new_surface_ids) {
                    new_refs += ",#" + std::to_string(sid);
                }
                data.insert(close_paren, new_refs);
            }
            out << data;
        } else {
            out << entity.data;
        }
        out << ";\n";
    }

    // Write new NurbsCurves (not in existing entities)
    size_t curve_idx = 0;
    for (const auto& [id, curve] : step.curves) {
        if (existing_curve_ids.count(id)) continue;

        int cv_count = curve.cv_count();
        int degree = curve.degree();

        // Write control points
        std::vector<int> cv_ids;
        for (int i = 0; i < cv_count; i++) {
            Point pt = curve.get_cv(i);
            out << "#" << next_id << "=CARTESIAN_POINT(''," << format_point(pt[0], pt[1], pt[2]) << ");\n";
            cv_ids.push_back(next_id++);
        }

        // Build knot multiplicities and unique knots
        std::vector<double> knots_unique;
        std::vector<int> mults;
        int knot_count = curve.knot_count();
        if (knot_count > 0) {
            double prev = curve.knot(0) - 1.0;
            for (int i = 0; i < knot_count; i++) {
                double k = curve.knot(i);
                if (i == 0 || std::abs(k - prev) > 1e-10) {
                    knots_unique.push_back(k);
                    mults.push_back(1);
                } else {
                    mults.back()++;
                }
                prev = k;
            }
            mults.front()++;
            mults.back()++;
        }

        // Write B_SPLINE_CURVE_WITH_KNOTS
        int curve_entity_id = new_curve_ids[curve_idx++];
        out << "#" << curve_entity_id << "=B_SPLINE_CURVE_WITH_KNOTS(''," << degree << ",(";
        for (size_t i = 0; i < cv_ids.size(); i++) {
            out << "#" << cv_ids[i];
            if (i < cv_ids.size() - 1) out << ",";
        }
        out << "),.UNSPECIFIED.,.F.,.F.,(";
        for (size_t i = 0; i < mults.size(); i++) {
            out << mults[i];
            if (i < mults.size() - 1) out << ",";
        }
        out << "),(";
        for (size_t i = 0; i < knots_unique.size(); i++) {
            out << format_double(knots_unique[i]);
            if (i < knots_unique.size() - 1) out << ",";
        }
        out << "),.UNSPECIFIED.);\n";
        next_id = curve_entity_id + 1;
    }

    // Write new NurbsSurfaces (not in existing entities)
    size_t surf_idx = 0;
    for (const auto& [id, surf] : step.surfaces) {
        if (existing_surface_ids.count(id)) continue;

        int cv_count_u = surf.cv_count(0);
        int cv_count_v = surf.cv_count(1);
        int degree_u = surf.degree(0);
        int degree_v = surf.degree(1);

        // Write control points
        std::vector<std::vector<int>> cv_ids(cv_count_u, std::vector<int>(cv_count_v));
        for (int i = 0; i < cv_count_u; i++) {
            for (int j = 0; j < cv_count_v; j++) {
                Point pt = surf.get_cv(i, j);
                out << "#" << next_id << "=CARTESIAN_POINT(''," << format_point(pt[0], pt[1], pt[2]) << ");\n";
                cv_ids[i][j] = next_id++;
            }
        }

        // Build U knot multiplicities
        std::vector<double> u_knots_unique;
        std::vector<int> u_mults;
        int u_knot_count = surf.knot_count(0);
        if (u_knot_count > 0) {
            double prev = surf.knot(0, 0) - 1.0;
            for (int i = 0; i < u_knot_count; i++) {
                double k = surf.knot(0, i);
                if (i == 0 || std::abs(k - prev) > 1e-10) {
                    u_knots_unique.push_back(k);
                    u_mults.push_back(1);
                } else {
                    u_mults.back()++;
                }
                prev = k;
            }
            u_mults.front()++;
            u_mults.back()++;
        }

        // Build V knot multiplicities
        std::vector<double> v_knots_unique;
        std::vector<int> v_mults;
        int v_knot_count = surf.knot_count(1);
        if (v_knot_count > 0) {
            double prev = surf.knot(1, 0) - 1.0;
            for (int i = 0; i < v_knot_count; i++) {
                double k = surf.knot(1, i);
                if (i == 0 || std::abs(k - prev) > 1e-10) {
                    v_knots_unique.push_back(k);
                    v_mults.push_back(1);
                } else {
                    v_mults.back()++;
                }
                prev = k;
            }
            v_mults.front()++;
            v_mults.back()++;
        }

        // Write B_SPLINE_SURFACE_WITH_KNOTS
        int surf_entity_id = new_surface_ids[surf_idx++];
        out << "#" << surf_entity_id << "=B_SPLINE_SURFACE_WITH_KNOTS(''," << degree_u << "," << degree_v << ",(";
        for (int i = 0; i < cv_count_u; i++) {
            out << "(";
            for (int j = 0; j < cv_count_v; j++) {
                out << "#" << cv_ids[i][j];
                if (j < cv_count_v - 1) out << ",";
            }
            out << ")";
            if (i < cv_count_u - 1) out << ",";
        }
        out << "),.UNSPECIFIED.,.F.,.F.,.F.,(";
        for (size_t i = 0; i < u_mults.size(); i++) {
            out << u_mults[i];
            if (i < u_mults.size() - 1) out << ",";
        }
        out << "),(";
        for (size_t i = 0; i < v_mults.size(); i++) {
            out << v_mults[i];
            if (i < v_mults.size() - 1) out << ",";
        }
        out << "),(";
        for (size_t i = 0; i < u_knots_unique.size(); i++) {
            out << format_double(u_knots_unique[i]);
            if (i < u_knots_unique.size() - 1) out << ",";
        }
        out << "),(";
        for (size_t i = 0; i < v_knots_unique.size(); i++) {
            out << format_double(v_knots_unique[i]);
            if (i < v_knots_unique.size() - 1) out << ",";
        }
        out << "),.UNSPECIFIED.);\n";
        next_id = surf_entity_id + 1;
    }

    // Create wrapper entities for new surfaces to make them visible
    if (!new_surface_ids.empty()) {
        // Find required references from existing entities
        int context_id = -1;      // GEOMETRIC_REPRESENTATION_CONTEXT or similar
        int axis_id = -1;         // AXIS2_PLACEMENT_3D
        int shape_rep_id = -1;    // Main SHAPE_REPRESENTATION

        for (const auto& entity : step.entities) {
            if (entity.data.find("GEOMETRIC_REPRESENTATION_CONTEXT") != std::string::npos && context_id < 0) {
                context_id = entity.id;
            }
            if (entity.type == "AXIS2_PLACEMENT_3D" && axis_id < 0) {
                axis_id = entity.id;
            }
            if (entity.type == "SHAPE_REPRESENTATION" && shape_rep_id < 0) {
                shape_rep_id = entity.id;
            }
        }

        if (context_id > 0 && axis_id > 0) {
            // Create GEOMETRIC_SET containing all new surfaces
            int geom_set_id = next_id++;
            out << "#" << geom_set_id << "=GEOMETRIC_SET('new_surfaces',(";
            for (size_t i = 0; i < new_surface_ids.size(); i++) {
                out << "#" << new_surface_ids[i];
                if (i < new_surface_ids.size() - 1) out << ",";
            }
            out << "));\n";

            // Create GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION
            int surf_rep_id = next_id++;
            out << "#" << surf_rep_id << "=GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION("
                << "'new_surface_rep',(#" << geom_set_id << ",#" << axis_id << "),#" << context_id << ");\n";

            // Create SHAPE_REPRESENTATION_RELATIONSHIP to link to main shape
            if (shape_rep_id > 0) {
                out << "#" << next_id++ << "=SHAPE_REPRESENTATION_RELATIONSHIP('','',#"
                    << shape_rep_id << ",#" << surf_rep_id << ");\n";
            }
        }
    }

    out << "ENDSEC;\n";
    out << "END-ISO-10303-21;\n";
    out.close();
}

} // namespace session_cpp
