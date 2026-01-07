#pragma once

#include "point.h"
#include "line.h"
#include "nurbscurve.h"
#include "nurbssurface.h"
#include <string>
#include <vector>
#include <unordered_map>

namespace session_cpp {

// Raw STEP entity for file structure preservation
struct StepEntity {
    int id;
    std::string type;
    std::string data;
};

// STEP file with session_cpp geometry
struct StepFile {
    std::string filename;
    std::string timestamp;
    std::string originating_system;
    std::string schema;

    // Raw entities (for structure preservation during write)
    std::vector<StepEntity> entities;
    std::unordered_map<int, size_t> entity_index;

    // Geometry using session_cpp types (indexed by STEP entity ID)
    std::unordered_map<int, Point> points;
    std::unordered_map<int, Line> lines;
    std::unordered_map<int, NurbsCurve> curves;
    std::unordered_map<int, NurbsSurface> surfaces;
};

class StepReader {
public:
    static StepFile read(const std::string& filepath);

private:
    StepFile file_;
    std::string content_;

    // Temporary storage during parsing (needed for reference resolution)
    struct TempPoint { double x, y, z; };
    struct TempDirection { double x, y, z; };
    struct TempVector { int dir_id; double mag; };
    std::unordered_map<int, TempPoint> temp_points_;
    std::unordered_map<int, TempDirection> temp_directions_;
    std::unordered_map<int, TempVector> temp_vectors_;

    void parse_header();
    void parse_data();
    void parse_entity(const std::string& line);
    void build_geometry();

    static std::string trim(const std::string& s);
    static std::vector<double> parse_double_list(const std::string& s);
    static std::vector<int> parse_int_list(const std::string& s);
    static std::vector<int> parse_ref_list(const std::string& s);
};

} // namespace session_cpp
