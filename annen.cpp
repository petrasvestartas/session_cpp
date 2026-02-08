#include "src/nurbssurface.h"
#include "src/point.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <filesystem>

using namespace session_cpp;

static std::string read_file(const std::string& path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static std::string join_entity(const std::string& raw, size_t start) {
    std::string result;
    for (size_t i = start; i < raw.size(); i++) {
        char c = raw[i];
        if (c == ';') break;
        if (c == '\n' || c == '\r') continue;
        result += c;
    }
    return result;
}

static std::vector<double> parse_double_list(const std::string& s, size_t start, size_t end) {
    std::vector<double> vals;
    size_t i = start;
    while (i < end) {
        while (i < end && (s[i] == ',' || s[i] == ' ' || s[i] == '(')) i++;
        if (i >= end || s[i] == ')') break;
        size_t j = i;
        while (j < end && s[j] != ',' && s[j] != ')') j++;
        vals.push_back(std::stod(s.substr(i, j - i)));
        i = j;
    }
    return vals;
}

static std::vector<int> parse_int_list(const std::string& s, size_t start, size_t end) {
    std::vector<int> vals;
    size_t i = start;
    while (i < end) {
        while (i < end && (s[i] == ',' || s[i] == ' ' || s[i] == '(')) i++;
        if (i >= end || s[i] == ')') break;
        size_t j = i;
        while (j < end && s[j] != ',' && s[j] != ')') j++;
        vals.push_back(std::stoi(s.substr(i, j - i)));
        i = j;
    }
    return vals;
}

static size_t find_matching_paren(const std::string& s, size_t open) {
    int depth = 0;
    for (size_t i = open; i < s.size(); i++) {
        if (s[i] == '(') depth++;
        else if (s[i] == ')') { depth--; if (depth == 0) return i; }
    }
    return std::string::npos;
}

// Parse CV grid: ((#id,#id,...),(#id,#id,...),...)
// Returns 2D array [row][col] of entity IDs
static std::vector<std::vector<int>> parse_cv_grid(const std::string& s, size_t start) {
    std::vector<std::vector<int>> grid;
    size_t outer_open = s.find('(', start);
    if (outer_open == std::string::npos) return grid;
    size_t outer_close = find_matching_paren(s, outer_open);

    size_t pos = outer_open + 1;
    while (pos < outer_close) {
        size_t row_open = s.find('(', pos);
        if (row_open == std::string::npos || row_open >= outer_close) break;
        size_t row_close = find_matching_paren(s, row_open);

        std::vector<int> row;
        size_t p = row_open + 1;
        while (p < row_close) {
            size_t hash = s.find('#', p);
            if (hash == std::string::npos || hash >= row_close) break;
            size_t num_start = hash + 1;
            size_t num_end = num_start;
            while (num_end < row_close && s[num_end] >= '0' && s[num_end] <= '9') num_end++;
            row.push_back(std::stoi(s.substr(num_start, num_end - num_start)));
            p = num_end;
        }
        grid.push_back(row);
        pos = row_close + 1;
    }
    return grid;
}

// Find the next parenthesized list after 'pos', return start of '(' and end of ')'
static std::pair<size_t, size_t> find_next_paren_list(const std::string& s, size_t pos) {
    size_t open = s.find('(', pos);
    if (open == std::string::npos) return {std::string::npos, std::string::npos};
    size_t close = find_matching_paren(s, open);
    return {open, close};
}

int main() {
    std::string stp = read_file("session_data/annen.stp");

    // Pass 1: extract CARTESIAN_POINT entities
    std::unordered_map<int, Point> points;
    {
        size_t pos = 0;
        while ((pos = stp.find("CARTESIAN_POINT(", pos)) != std::string::npos) {
            // find entity id: scan backwards for #
            size_t eq = stp.rfind('#', pos);
            size_t id_start = eq + 1;
            size_t id_end = id_start;
            while (id_end < pos && stp[id_end] >= '0' && stp[id_end] <= '9') id_end++;
            int id = std::stoi(stp.substr(id_start, id_end - id_start));

            // find coordinates: second parenthesized list
            size_t paren1 = stp.find('(', pos + 16); // skip "CARTESIAN_POINT("
            size_t paren2 = stp.find('(', paren1);
            size_t paren2_close = stp.find(')', paren2);
            auto coords = parse_double_list(stp, paren2 + 1, paren2_close);
            if (coords.size() >= 3) {
                points[id] = Point(coords[0], coords[1], coords[2]);
            }
            pos = paren2_close + 1;
        }
    }
    std::cout << "Parsed " << points.size() << " CARTESIAN_POINT entities" << std::endl;

    // Pass 2: extract B_SPLINE_SURFACE_WITH_KNOTS
    std::vector<NurbsSurface> surfaces;
    {
        size_t pos = 0;
        while ((pos = stp.find("B_SPLINE_SURFACE_WITH_KNOTS(", pos)) != std::string::npos) {
            size_t eq = stp.rfind('#', pos);
            int entity_id = std::stoi(stp.substr(eq + 1, pos - eq - 2));

            std::string entity = join_entity(stp, pos);

            // Parse: B_SPLINE_SURFACE_WITH_KNOTS('',degree_u,degree_v,((cvs...)),
            //   .UNSPECIFIED.,.F.,.F.,.F.,
            //   (mult_u),(mult_v),(knots_u),(knots_v),.UNSPECIFIED.)

            // Find first '(' after "B_SPLINE_SURFACE_WITH_KNOTS"
            size_t p = entity.find('(');
            // Skip name string: find closing quote after opening
            size_t name_end = entity.find('\'', entity.find('\'', p + 1) + 1);

            // After name, find degree_u and degree_v
            size_t comma1 = entity.find(',', name_end + 1);
            size_t comma2 = entity.find(',', comma1 + 1);
            size_t comma3 = entity.find(',', comma2 + 1);
            int degree_u = std::stoi(entity.substr(comma1 + 1, comma2 - comma1 - 1));
            int degree_v = std::stoi(entity.substr(comma2 + 1, comma3 - comma2 - 1));

            // Parse CV grid starting at comma3+1
            auto cv_grid = parse_cv_grid(entity, comma3 + 1);
            int cv_count_u = (int)cv_grid.size();
            int cv_count_v = cv_count_u > 0 ? (int)cv_grid[0].size() : 0;

            // Skip past CV grid and the .UNSPECIFIED.,.F.,.F.,.F. flags
            // Find the closing )) of the CV grid
            size_t cv_outer = entity.find("((", comma3);
            size_t cv_end = find_matching_paren(entity, cv_outer);

            // After CV grid close, skip flags: .UNSPECIFIED.,.F.,.F.,.F.,
            // Find 4 parenthesized lists: mult_u, mult_v, knots_u, knots_v
            size_t search_from = cv_end + 1;

            // Skip past the flags to find first '(' of mult_u
            auto [mu_start, mu_end] = find_next_paren_list(entity, search_from);

            // Find all remaining paren lists
            std::vector<std::pair<size_t, size_t>> paren_lists;
            size_t scan = search_from;
            while (paren_lists.size() < 4) {
                auto [s, e] = find_next_paren_list(entity, scan);
                if (s == std::string::npos) break;
                paren_lists.push_back({s, e});
                scan = e + 1;
            }

            if (paren_lists.size() < 4) {
                pos++;
                continue;
            }

            auto mult_u = parse_int_list(entity, paren_lists[0].first + 1, paren_lists[0].second);
            auto mult_v = parse_int_list(entity, paren_lists[1].first + 1, paren_lists[1].second);
            auto knots_u_vals = parse_double_list(entity, paren_lists[2].first + 1, paren_lists[2].second);
            auto knots_v_vals = parse_double_list(entity, paren_lists[3].first + 1, paren_lists[3].second);

            // Expand knot multiplicities
            std::vector<double> knots_u_full, knots_v_full;
            for (size_t i = 0; i < knots_u_vals.size() && i < mult_u.size(); i++)
                for (int j = 0; j < mult_u[i]; j++)
                    knots_u_full.push_back(knots_u_vals[i]);
            for (size_t i = 0; i < knots_v_vals.size() && i < mult_v.size(); i++)
                for (int j = 0; j < mult_v[i]; j++)
                    knots_v_full.push_back(knots_v_vals[i]);

            // STEP stores order+cv_count knots, OpenNURBS uses order+cv_count-2
            // Strip first and last knot from STEP's expanded vector
            int order_u = degree_u + 1;
            int order_v = degree_v + 1;
            int expected_knots_u = order_u + cv_count_u - 2;
            int expected_knots_v = order_v + cv_count_v - 2;

            std::vector<double> knots_u(knots_u_full.begin() + 1, knots_u_full.end() - 1);
            std::vector<double> knots_v(knots_v_full.begin() + 1, knots_v_full.end() - 1);

            if ((int)knots_u.size() != expected_knots_u || (int)knots_v.size() != expected_knots_v) {
                std::cout << "WARNING: knot count mismatch for #" << entity_id
                          << " u:" << knots_u.size() << "/" << expected_knots_u
                          << " v:" << knots_v.size() << "/" << expected_knots_v << std::endl;
                pos++;
                continue;
            }

            NurbsSurface surf;
            surf.create_raw(3, false, order_u, order_v, cv_count_u, cv_count_v);
            surf.name = "stp_surface_" + std::to_string(entity_id);

            // Set knots
            for (int i = 0; i < expected_knots_u; i++)
                surf.set_knot(0, i, knots_u[i]);
            for (int i = 0; i < expected_knots_v; i++)
                surf.set_knot(1, i, knots_v[i]);

            // Set CVs
            for (int i = 0; i < cv_count_u; i++) {
                for (int j = 0; j < cv_count_v; j++) {
                    int ref_id = cv_grid[i][j];
                    auto it = points.find(ref_id);
                    if (it != points.end()) {
                        surf.set_cv(i, j, it->second);
                    }
                }
            }

            std::cout << "#" << entity_id << ": " << surf.str()
                      << " valid=" << surf.is_valid() << std::endl;
            surfaces.push_back(std::move(surf));
            pos++;
        }
    }

    std::cout << "\nLoaded " << surfaces.size() << " B_SPLINE_SURFACE_WITH_KNOTS" << std::endl;

    // Dump as JSON
    std::filesystem::create_directories("session_data/annen_surfaces");
    for (size_t i = 0; i < surfaces.size(); i++) {
        std::string path = "session_data/annen_surfaces/surface_" + std::to_string(i) + ".json";
        surfaces[i].json_dump(path);
    }
    std::cout << "Wrote " << surfaces.size() << " JSON files to session_data/annen_surfaces/" << std::endl;

    return 0;
}
