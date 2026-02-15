#!/bin/bash
# Script to add get_ prefix to NurbsSurface functions

set -e

echo "Adding get_ prefix to NurbsSurface functions..."

# nurbssurface.h - update declarations
sed -i 's/\bdimension\(\) const { return m_dim; }/get_dimension() const { return m_dim; }/g' src/nurbssurface.h
sed -i 's/\bint order(int dir) const;/int get_order(int dir) const;/g' src/nurbssurface.h
sed -i 's/\bint degree(int dir) const;/int get_degree(int dir) const;/g' src/nurbssurface.h
sed -i 's/\bint cv_count(int dir) const;/int get_cv_count(int dir) const;/g' src/nurbssurface.h
sed -i 's/\bint cv_count() const;/int get_cv_count() const;/g' src/nurbssurface.h
sed -i 's/\bint cv_size() const;/int get_cv_size() const;/g' src/nurbssurface.h
sed -i 's/\bint knot_count(int dir) const;/int get_knot_count(int dir) const;/g' src/nurbssurface.h
sed -i 's/\bint span_count(int dir) const;/int get_span_count(int dir) const;/g' src/nurbssurface.h
sed -i 's/\bdouble knot(int dir, int knot_index) const;/double get_knot(int dir, int knot_index) const;/g' src/nurbssurface.h
sed -i 's/\bint knot_multiplicity(int dir, int knot_index) const;/int get_knot_multiplicity(int dir, int knot_index) const;/g' src/nurbssurface.h
sed -i 's/\bstd::pair<double, double> domain(int dir) const;/std::pair<double, double> get_domain(int dir) const;/g' src/nurbssurface.h

echo "Updated nurbssurface.h"

# nurbssurface.cpp - update implementations
sed -i 's/\bNurbsSurface::order(/NurbsSurface::get_order(/g' src/nurbssurface.cpp
sed -i 's/\bNurbsSurface::degree(/NurbsSurface::get_degree(/g' src/nurbssurface.cpp
sed -i 's/\bNurbsSurface::cv_count(int dir)/NurbsSurface::get_cv_count(int dir)/g' src/nurbssurface.cpp
sed -i 's/\bNurbsSurface::cv_count()/NurbsSurface::get_cv_count()/g' src/nurbssurface.cpp
sed -i 's/\bNurbsSurface::cv_size(/NurbsSurface::get_cv_size(/g' src/nurbssurface.cpp
sed -i 's/\bNurbsSurface::knot_count(/NurbsSurface::get_knot_count(/g' src/nurbssurface.cpp
sed -i 's/\bNurbsSurface::span_count(/NurbsSurface::get_span_count(/g' src/nurbssurface.cpp
sed -i 's/\bNurbsSurface::knot(/NurbsSurface::get_knot(/g' src/nurbssurface.cpp
sed -i 's/\bNurbsSurface::knot_multiplicity(/NurbsSurface::get_knot_multiplicity(/g' src/nurbssurface.cpp
sed -i 's/\bNurbsSurface::domain(/NurbsSurface::get_domain(/g' src/nurbssurface.cpp

echo "Updated nurbssurface.cpp"

# Update internal callers in nurbssurface.cpp
sed -i 's/knot_count(/get_knot_count(/g' src/nurbssurface.cpp
sed -i 's/span_count(/get_span_count(/g' src/nurbssurface.cpp
sed -i 's/cv_count(/get_cv_count(/g' src/nurbssurface.cpp
sed -i 's/order(/get_order(/g' src/nurbssurface.cpp
sed -i 's/degree(/get_degree(/g' src/nurbssurface.cpp
sed -i 's/knot(/get_knot(/g' src/nurbssurface.cpp
sed -i 's/domain(/get_domain(/g' src/nurbssurface.cpp
sed -i 's/cv_size(/get_cv_size(/g' src/nurbssurface.cpp

echo "Updated internal callers in nurbssurface.cpp"

# Update all test files - nurbssurface_test.cpp
sed -i 's/\.degree(/\.get_degree(/g' src/nurbssurface_test.cpp
sed -i 's/\.order(/\.get_order(/g' src/nurbssurface_test.cpp
sed -i 's/\.cv_count(/\.get_cv_count(/g' src/nurbssurface_test.cpp
sed -i 's/\.dimension(/\.get_dimension(/g' src/nurbssurface_test.cpp
sed -i 's/\.knot_count(/\.get_knot_count(/g' src/nurbssurface_test.cpp
sed -i 's/\.span_count(/\.get_span_count(/g' src/nurbssurface_test.cpp
sed -i 's/\.knot(/\.get_knot(/g' src/nurbssurface_test.cpp
sed -i 's/\.knot_multiplicity(/\.get_knot_multiplicity(/g' src/nurbssurface_test.cpp
sed -i 's/\.domain(/\.get_domain(/g' src/nurbssurface_test.cpp
sed -i 's/\.cv_size(/\.get_cv_size(/g' src/nurbssurface_test.cpp

echo "Updated nurbssurface_test.cpp"

# Update other source files that call NurbsSurface
for file in src/*.cpp; do
    if [[ "$file" != *"nurbssurface"* ]] && [[ "$file" != *"nurbscurve"* ]]; then
        sed -i 's/\.degree(/\.get_degree(/g' "$file"
        sed -i 's/\.order(/\.get_order(/g' "$file"
        sed -i 's/\.cv_count(/\.get_cv_count(/g' "$file"
        sed -i 's/\.dimension(/\.get_dimension(/g' "$file"
        sed -i 's/\.knot_count(/\.get_knot_count(/g' "$file"
        sed -i 's/\.span_count(/\.get_span_count(/g' "$file"
        sed -i 's/\.knot(/\.get_knot(/g' "$file"
        sed -i 's/\.knot_multiplicity(/\.get_knot_multiplicity(/g' "$file"
        sed -i 's/\.domain(/\.get_domain(/g' "$file"
        sed -i 's/\.cv_size(/\.get_cv_size(/g' "$file"
    fi
done

echo "Updated other source files"

# Update main files
sed -i 's/\.degree(/\.get_degree(/g' main_1.cpp
sed -i 's/\.order(/\.get_order(/g' main_1.cpp
sed -i 's/\.cv_count(/\.get_cv_count(/g' main_1.cpp

echo "Updated main files"

echo "Done!"
