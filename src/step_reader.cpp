#include "step_reader.h"
#include <fstream>
#include <sstream>
#include <regex>
#include <algorithm>
#include <cctype>

namespace session_cpp {

StepFile StepReader::read(const std::string& filepath) {
    StepReader reader;

    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open STEP file: " + filepath);
    }

    reader.content_ = std::string(
        std::istreambuf_iterator<char>(file),
        std::istreambuf_iterator<char>()
    );
    file.close();

    // Remove newlines for easier parsing
    std::string consolidated;
    consolidated.reserve(reader.content_.size());
    for (char c : reader.content_) {
        if (c != '\n' && c != '\r') {
            consolidated += c;
        }
    }
    reader.content_ = consolidated;

    reader.parse_header();
    reader.parse_data();
    reader.build_geometry();

    return reader.file_;
}

void StepReader::parse_header() {
    size_t header_start = content_.find("HEADER;");
    size_t header_end = content_.find("ENDSEC;");
    if (header_start == std::string::npos || header_end == std::string::npos) return;

    std::string header = content_.substr(header_start, header_end - header_start);

    // Parse FILE_SCHEMA
    size_t schema_start = header.find("FILE_SCHEMA");
    if (schema_start != std::string::npos) {
        size_t p1 = header.find("'", schema_start);
        size_t p2 = header.find("'", p1 + 1);
        if (p1 != std::string::npos && p2 != std::string::npos) {
            file_.schema = header.substr(p1 + 1, p2 - p1 - 1);
        }
    }

    // Parse FILE_NAME
    size_t fn_start = header.find("FILE_NAME");
    if (fn_start != std::string::npos) {
        std::vector<std::string> quoted;
        size_t pos = fn_start;
        while (true) {
            size_t q1 = header.find("'", pos);
            if (q1 == std::string::npos) break;
            size_t q2 = header.find("'", q1 + 1);
            if (q2 == std::string::npos) break;
            quoted.push_back(header.substr(q1 + 1, q2 - q1 - 1));
            pos = q2 + 1;
            if (quoted.size() >= 7) break;
        }
        if (quoted.size() >= 1) file_.filename = quoted[0];
        if (quoted.size() >= 2) file_.timestamp = quoted[1];
        if (quoted.size() >= 6) file_.originating_system = quoted[5];
    }
}

void StepReader::parse_data() {
    size_t data_start = content_.find("DATA;");
    size_t data_end = content_.rfind("ENDSEC;");
    if (data_start == std::string::npos || data_end == std::string::npos) return;

    std::string data = content_.substr(data_start + 5, data_end - data_start - 5);

    size_t pos = 0;
    while (pos < data.size()) {
        size_t end = data.find(';', pos);
        if (end == std::string::npos) break;

        std::string entity = trim(data.substr(pos, end - pos));
        if (!entity.empty() && entity[0] == '#') {
            parse_entity(entity);
        }
        pos = end + 1;
    }
}

void StepReader::parse_entity(const std::string& line) {
    size_t eq = line.find('=');
    if (eq == std::string::npos) return;

    int id;
    try {
        id = std::stoi(trim(line.substr(1, eq - 1)));
    } catch (...) {
        return;
    }

    std::string rest = trim(line.substr(eq + 1));

    // Store raw entity
    StepEntity raw;
    raw.id = id;
    raw.data = rest;
    size_t paren = rest.find('(');
    raw.type = (paren != std::string::npos) ? trim(rest.substr(0, paren)) : rest;

    file_.entity_index[id] = file_.entities.size();
    file_.entities.push_back(raw);

    // Parse CARTESIAN_POINT -> temp storage
    if (rest.find("CARTESIAN_POINT") == 0) {
        size_t p1 = rest.find(",(");
        size_t p2 = rest.rfind("))");
        if (p1 != std::string::npos && p2 != std::string::npos) {
            auto coords = parse_double_list(rest.substr(p1 + 2, p2 - p1 - 2));
            if (coords.size() >= 3) {
                temp_points_[id] = {coords[0], coords[1], coords[2]};
            }
        }
    }
    // Parse DIRECTION -> temp storage
    else if (rest.find("DIRECTION") == 0) {
        size_t p1 = rest.find(",(");
        size_t p2 = rest.rfind("))");
        if (p1 != std::string::npos && p2 != std::string::npos) {
            auto coords = parse_double_list(rest.substr(p1 + 2, p2 - p1 - 2));
            if (coords.size() >= 3) {
                temp_directions_[id] = {coords[0], coords[1], coords[2]};
            }
        }
    }
    // Parse VECTOR -> temp storage
    else if (rest.find("VECTOR") == 0 && rest.find("B_SPLINE") == std::string::npos) {
        auto refs = parse_ref_list(rest);
        size_t last_comma = rest.rfind(',');
        size_t last_paren = rest.rfind(')');
        if (!refs.empty() && last_comma != std::string::npos) {
            try {
                double mag = std::stod(trim(rest.substr(last_comma + 1, last_paren - last_comma - 1)));
                temp_vectors_[id] = {refs[0], mag};
            } catch (...) {}
        }
    }
}

void StepReader::build_geometry() {
    // Convert temp points to session_cpp Points
    for (const auto& [id, tp] : temp_points_) {
        file_.points[id] = Point(tp.x, tp.y, tp.z);
    }

    // Build Lines from LINE entities
    for (const auto& entity : file_.entities) {
        if (entity.type == "LINE" && entity.data.find("B_SPLINE") == std::string::npos) {
            auto refs = parse_ref_list(entity.data);
            if (refs.size() >= 2) {
                int pt_id = refs[0];
                int vec_id = refs[1];
                auto pt_it = temp_points_.find(pt_id);
                auto vec_it = temp_vectors_.find(vec_id);
                if (pt_it != temp_points_.end() && vec_it != temp_vectors_.end()) {
                    auto dir_it = temp_directions_.find(vec_it->second.dir_id);
                    if (dir_it != temp_directions_.end()) {
                        double x0 = pt_it->second.x;
                        double y0 = pt_it->second.y;
                        double z0 = pt_it->second.z;
                        double mag = vec_it->second.mag;
                        double x1 = x0 + dir_it->second.x * mag;
                        double y1 = y0 + dir_it->second.y * mag;
                        double z1 = z0 + dir_it->second.z * mag;
                        file_.lines[entity.id] = Line(x0, y0, z0, x1, y1, z1);
                    }
                }
            }
        }
    }

    // Build NurbsCurves from B_SPLINE_CURVE_WITH_KNOTS entities
    for (const auto& entity : file_.entities) {
        if (entity.data.find("B_SPLINE_CURVE_WITH_KNOTS") == std::string::npos) continue;

        bool is_rational = entity.data.find("RATIONAL") != std::string::npos;
        bool is_complex = entity.data[0] == '(' && entity.data.find("B_SPLINE_CURVE(") != std::string::npos;

        int degree = 0;
        std::vector<int> cp_ids;
        std::vector<int> mults;
        std::vector<double> knots;

        if (is_complex) {
            // Complex entity format
            size_t bc_pos = entity.data.find("B_SPLINE_CURVE(");
            if (bc_pos == std::string::npos) continue;
            std::string params = entity.data.substr(bc_pos + 15);
            int depth = 1;
            size_t end = 0;
            for (size_t i = 0; i < params.size() && depth > 0; i++) {
                if (params[i] == '(') depth++;
                else if (params[i] == ')') depth--;
                if (depth == 0) end = i;
            }
            params = params.substr(0, end);

            size_t c1 = params.find(',');
            if (c1 == std::string::npos) continue;
            try { degree = std::stoi(trim(params.substr(0, c1))); } catch (...) { continue; }

            size_t cp_start = params.find('(');
            size_t cp_end = params.find(')', cp_start);
            if (cp_start != std::string::npos && cp_end != std::string::npos) {
                cp_ids = parse_ref_list(params.substr(cp_start + 1, cp_end - cp_start - 1));
            }

            // Knots from B_SPLINE_CURVE_WITH_KNOTS
            size_t knot_pos = entity.data.find("B_SPLINE_CURVE_WITH_KNOTS(");
            if (knot_pos != std::string::npos) {
                std::string kp = entity.data.substr(knot_pos + 26);
                depth = 1; end = 0;
                for (size_t i = 0; i < kp.size() && depth > 0; i++) {
                    if (kp[i] == '(') depth++;
                    else if (kp[i] == ')') depth--;
                    if (depth == 0) end = i;
                }
                kp = kp.substr(0, end);

                size_t m_start = kp.find('(');
                size_t m_end = kp.find(')', m_start);
                if (m_start != std::string::npos && m_end != std::string::npos) {
                    mults = parse_int_list(kp.substr(m_start + 1, m_end - m_start - 1));
                }
                size_t k_start = kp.find('(', m_end + 1);
                size_t k_end = kp.find(')', k_start);
                if (k_start != std::string::npos && k_end != std::string::npos) {
                    knots = parse_double_list(kp.substr(k_start + 1, k_end - k_start - 1));
                }
            }
        } else {
            // Simple entity format
            size_t paren = entity.data.find('(');
            if (paren == std::string::npos) continue;
            std::string params = entity.data.substr(paren + 1);
            if (!params.empty() && params.back() == ')') params.pop_back();

            size_t c1 = params.find(',');
            size_t c2 = params.find(',', c1 + 1);
            if (c1 == std::string::npos || c2 == std::string::npos) continue;
            try { degree = std::stoi(trim(params.substr(c1 + 1, c2 - c1 - 1))); } catch (...) { continue; }

            size_t cp_start = params.find('(', c2);
            size_t cp_end = params.find(')', cp_start);
            if (cp_start != std::string::npos && cp_end != std::string::npos) {
                cp_ids = parse_ref_list(params.substr(cp_start + 1, cp_end - cp_start - 1));
            }

            size_t m_start = params.find('(', cp_end + 1);
            size_t m_end = params.find(')', m_start);
            if (m_start != std::string::npos && m_end != std::string::npos) {
                mults = parse_int_list(params.substr(m_start + 1, m_end - m_start - 1));
            }

            size_t k_start = params.find('(', m_end + 1);
            size_t k_end = params.find(')', k_start);
            if (k_start != std::string::npos && k_end != std::string::npos) {
                knots = parse_double_list(params.substr(k_start + 1, k_end - k_start - 1));
            }
        }

        // Build NurbsCurve
        int order = degree + 1;
        int cv_count = static_cast<int>(cp_ids.size());
        if (cv_count == 0) continue;

        NurbsCurve curve(3, is_rational, order, cv_count);

        for (int i = 0; i < cv_count; i++) {
            auto it = temp_points_.find(cp_ids[i]);
            if (it != temp_points_.end()) {
                curve.set_cv(i, Point(it->second.x, it->second.y, it->second.z));
            }
        }

        // Expand and set knots
        std::vector<double> expanded;
        for (size_t i = 0; i < knots.size() && i < mults.size(); i++) {
            for (int m = 0; m < mults[i]; m++) {
                expanded.push_back(knots[i]);
            }
        }
        int expected = order + cv_count - 2;
        for (int i = 0; i < expected && i + 1 < static_cast<int>(expanded.size()); i++) {
            curve.set_knot(i, expanded[i + 1]);
        }

        if (curve.is_valid()) {
            file_.curves[entity.id] = curve;
        }
    }

    // Build NurbsSurfaces from B_SPLINE_SURFACE_WITH_KNOTS entities
    for (const auto& entity : file_.entities) {
        if (entity.data.find("B_SPLINE_SURFACE_WITH_KNOTS") == std::string::npos) continue;

        bool is_rational = entity.data.find("RATIONAL") != std::string::npos;
        bool is_complex = entity.data[0] == '(' && entity.data.find("B_SPLINE_SURFACE(") != std::string::npos;

        int degree_u = 0, degree_v = 0;
        std::vector<std::vector<int>> cp_ids;
        std::vector<int> u_mults, v_mults;
        std::vector<double> u_knots, v_knots;

        std::string params, knot_params;

        if (is_complex) {
            size_t bs_pos = entity.data.find("B_SPLINE_SURFACE(");
            if (bs_pos == std::string::npos) continue;
            params = entity.data.substr(bs_pos + 17);
            int depth = 1;
            size_t end = 0;
            for (size_t i = 0; i < params.size() && depth > 0; i++) {
                if (params[i] == '(') depth++;
                else if (params[i] == ')') depth--;
                if (depth == 0) end = i;
            }
            params = params.substr(0, end);

            size_t knot_pos = entity.data.find("B_SPLINE_SURFACE_WITH_KNOTS(");
            if (knot_pos != std::string::npos) {
                knot_params = entity.data.substr(knot_pos + 28);
                depth = 1; end = 0;
                for (size_t i = 0; i < knot_params.size() && depth > 0; i++) {
                    if (knot_params[i] == '(') depth++;
                    else if (knot_params[i] == ')') depth--;
                    if (depth == 0) end = i;
                }
                knot_params = knot_params.substr(0, end);
            }
        } else {
            size_t paren = entity.data.find('(');
            if (paren == std::string::npos) continue;
            params = entity.data.substr(paren + 1);
            if (!params.empty() && params.back() == ')') params.pop_back();
        }

        // Parse degrees
        size_t c1 = params.find(',');
        size_t c2 = params.find(',', c1 + 1);
        if (c1 == std::string::npos || c2 == std::string::npos) continue;

        try {
            if (is_complex) {
                degree_u = std::stoi(trim(params.substr(0, c1)));
                degree_v = std::stoi(trim(params.substr(c1 + 1, c2 - c1 - 1)));
            } else {
                size_t c3 = params.find(',', c2 + 1);
                if (c3 == std::string::npos) continue;
                degree_u = std::stoi(trim(params.substr(c1 + 1, c2 - c1 - 1)));
                degree_v = std::stoi(trim(params.substr(c2 + 1, c3 - c2 - 1)));
            }
        } catch (...) { continue; }

        // Parse 2D control points
        size_t cp_start = params.find("((");
        if (cp_start == std::string::npos) continue;

        int depth = 0;
        size_t cp_end = cp_start;
        for (size_t i = cp_start; i < params.size(); i++) {
            if (params[i] == '(') depth++;
            else if (params[i] == ')') {
                depth--;
                if (depth == 0) { cp_end = i; break; }
            }
        }

        std::string cp_str = params.substr(cp_start, cp_end - cp_start + 1);
        size_t pos = 0;
        while (pos < cp_str.size()) {
            size_t row_start = cp_str.find('(', pos);
            if (row_start == std::string::npos) break;
            if (row_start > 0 && cp_str[row_start - 1] == '(') {
                row_start = cp_str.find('(', row_start + 1);
                if (row_start == std::string::npos) break;
            }
            size_t row_end = cp_str.find(')', row_start);
            if (row_end == std::string::npos) break;

            auto refs = parse_ref_list(cp_str.substr(row_start + 1, row_end - row_start - 1));
            if (!refs.empty()) cp_ids.push_back(refs);
            pos = row_end + 1;
        }

        // Parse knots
        std::string rest = is_complex ? knot_params : params.substr(cp_end + 1);
        if (!is_complex) {
            int skip = 0;
            size_t spos = 0;
            while (skip < 4 && spos < rest.size()) {
                if (rest[spos] == ',') skip++;
                spos++;
            }
            rest = rest.substr(spos);
        }

        size_t um_start = rest.find('(');
        size_t um_end = rest.find(')', um_start);
        if (um_start != std::string::npos && um_end != std::string::npos)
            u_mults = parse_int_list(rest.substr(um_start + 1, um_end - um_start - 1));

        size_t vm_start = rest.find('(', um_end + 1);
        size_t vm_end = rest.find(')', vm_start);
        if (vm_start != std::string::npos && vm_end != std::string::npos)
            v_mults = parse_int_list(rest.substr(vm_start + 1, vm_end - vm_start - 1));

        size_t uk_start = rest.find('(', vm_end + 1);
        size_t uk_end = rest.find(')', uk_start);
        if (uk_start != std::string::npos && uk_end != std::string::npos)
            u_knots = parse_double_list(rest.substr(uk_start + 1, uk_end - uk_start - 1));

        size_t vk_start = rest.find('(', uk_end + 1);
        size_t vk_end = rest.find(')', vk_start);
        if (vk_start != std::string::npos && vk_end != std::string::npos)
            v_knots = parse_double_list(rest.substr(vk_start + 1, vk_end - vk_start - 1));

        // Build NurbsSurface
        int order_u = degree_u + 1;
        int order_v = degree_v + 1;
        int cv_count_u = static_cast<int>(cp_ids.size());
        int cv_count_v = cv_count_u > 0 ? static_cast<int>(cp_ids[0].size()) : 0;
        if (cv_count_u == 0 || cv_count_v == 0) continue;

        NurbsSurface surf(3, is_rational, order_u, order_v, cv_count_u, cv_count_v);

        for (int i = 0; i < cv_count_u; i++) {
            for (int j = 0; j < cv_count_v && j < static_cast<int>(cp_ids[i].size()); j++) {
                auto it = temp_points_.find(cp_ids[i][j]);
                if (it != temp_points_.end()) {
                    surf.set_cv(i, j, Point(it->second.x, it->second.y, it->second.z));
                }
            }
        }

        // Expand and set U knots
        std::vector<double> u_exp;
        for (size_t i = 0; i < u_knots.size() && i < u_mults.size(); i++) {
            for (int m = 0; m < u_mults[i]; m++) u_exp.push_back(u_knots[i]);
        }
        int expected_u = order_u + cv_count_u - 2;
        for (int i = 0; i < expected_u && i + 1 < static_cast<int>(u_exp.size()); i++) {
            surf.set_knot(0, i, u_exp[i + 1]);
        }

        // Expand and set V knots
        std::vector<double> v_exp;
        for (size_t i = 0; i < v_knots.size() && i < v_mults.size(); i++) {
            for (int m = 0; m < v_mults[i]; m++) v_exp.push_back(v_knots[i]);
        }
        int expected_v = order_v + cv_count_v - 2;
        for (int i = 0; i < expected_v && i + 1 < static_cast<int>(v_exp.size()); i++) {
            surf.set_knot(1, i, v_exp[i + 1]);
        }

        if (surf.is_valid()) {
            file_.surfaces[entity.id] = surf;
        }
    }
}

std::string StepReader::trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\n\r");
    return s.substr(start, end - start + 1);
}

std::vector<double> StepReader::parse_double_list(const std::string& s) {
    std::vector<double> result;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item = trim(item);
        if (!item.empty()) {
            try { result.push_back(std::stod(item)); }
            catch (...) {}
        }
    }
    return result;
}

std::vector<int> StepReader::parse_int_list(const std::string& s) {
    std::vector<int> result;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item = trim(item);
        if (!item.empty()) {
            try { result.push_back(std::stoi(item)); }
            catch (...) {}
        }
    }
    return result;
}

std::vector<int> StepReader::parse_ref_list(const std::string& s) {
    std::vector<int> result;
    std::regex ref_pattern("#(\\d+)");
    auto begin = std::sregex_iterator(s.begin(), s.end(), ref_pattern);
    auto end = std::sregex_iterator();
    for (auto it = begin; it != end; ++it) {
        result.push_back(std::stoi((*it)[1].str()));
    }
    return result;
}

} // namespace session_cpp
