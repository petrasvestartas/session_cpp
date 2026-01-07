#pragma once

#include "step_reader.h"
#include <string>

namespace session_cpp {

class StepWriter {
public:
    // Write a StepFile (preserves structure, updates modified points)
    static void write(const std::string& filepath, const StepFile& step);

private:
    static std::string format_double(double v);
    static std::string format_point(double x, double y, double z);
};

} // namespace session_cpp
