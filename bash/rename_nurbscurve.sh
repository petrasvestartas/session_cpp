#!/bin/bash
# Script to add get_ prefix to NurbsCurve functions

set -e

echo "Adding get_ prefix to NurbsCurve functions..."

# nurbscurve.h - update declarations
sed -i 's/\bint order() const { return m_order; }/int get_order() const { return m_order; }/g' src/nurbscurve.h
sed -i 's/\bint degree() const { return m_order - 1; }/int get_degree() const { return m_order - 1; }/g' src/nurbscurve.h
sed -i 's/\bint cv_count() const { return m_cv_count; }/int get_cv_count() const { return m_cv_count; }/g' src/nurbscurve.h
sed -i 's/\bint dimension() const { return m_dim; }/int get_dimension() const { return m_dim; }/g' src/nurbscurve.h
sed -i 's/\bint cv_size() const { return m_is_rat \\? (m_dim + 1) : m_dim; }/int get_cv_size() const { return m_is_rat \\? (m_dim + 1) : m_dim; }/g' src/nurbscurve.h
sed -i 's/\bint knot_count() const { return static_cast<int>(m_knot.size()); }/int get_knot_count() const { return static_cast<int>(m_knot.size()); }/g' src/nurbscurve.h
sed -i 's/\bint span_count() const { return knot_count() - order() + 1; }/int get_span_count() const { return get_knot_count() - get_order() + 1; }/g' src/nurbscurve.h

echo "Updated nurbscurve.h declarations"

# Update non-inline function declarations in nurbscurve.h
sed -i 's/\bint order(int span_index, int dir) const;/int get_order(int span_index, int dir) const;/g' src/nurbscurve.h
sed -i 's/\bdouble knot(int knot_index) const;/double get_knot(int knot_index) const;/g' src/nurbscurve.h
sed -i 's/\bint knot_multiplicity(int knot_index) const;/int get_knot_multiplicity(int knot_index) const;/g' src/nurbscurve.h
sed -i 's/\bdouble weight(int cv_index) const;/double get_weight(int cv_index) const;/g' src/nurbscurve.h
sed -i 's/\bstd::pair<double, double> domain() const;/std::pair<double, double> get_domain() const;/g' src/nurbscurve.h
sed -i 's/\bdouble domain_start() const;/double get_domain_start() const;/g' src/nurbscurve.h
sed -i 's/\bdouble domain_end() const;/double get_domain_end() const;/g' src/nurbscurve.h
sed -i 's/\bdouble domain_middle() const;/double get_domain_middle() const;/g' src/nurbscurve.h

echo "Updated nurbscurve.h declarations (non-inline)"

# nurbscurve.cpp - update implementations
sed -i 's/\bNurbsCurve::order(/NurbsCurve::get_order(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::degree(/NurbsCurve::get_degree(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::cv_count(/NurbsCurve::get_cv_count(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::dimension(/NurbsCurve::get_dimension(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::cv_size(/NurbsCurve::get_cv_size(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::knot_count(/NurbsCurve::get_knot_count(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::span_count(/NurbsCurve::get_span_count(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::knot(/NurbsCurve::get_knot(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::knot_multiplicity(/NurbsCurve::get_knot_multiplicity(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::weight(/NurbsCurve::get_weight(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::domain(/NurbsCurve::get_domain(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::domain_start(/NurbsCurve::get_domain_start(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::domain_end(/NurbsCurve::get_domain_end(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::domain_middle(/NurbsCurve::get_domain_middle(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::superfluous_knot(/NurbsCurve::get_superfluous_knot(/g' src/nurbscurve.cpp
sed -i 's/\bNurbsCurve::set_weight(/NurbsCurve::set_weight(/g' src/nurbscurve.cpp

echo "Updated nurbscurve.cpp implementations"

# Update internal callers in nurbscurve.cpp
sed -i 's/\border(/get_order(/g' src/nurbscurve.cpp
sed -i 's/\bdegree(/get_degree(/g' src/nurbscurve.cpp
sed -i 's/\bcv_count(/get_cv_count(/g' src/nurbscurve.cpp
sed -i 's/\bdimension(/get_dimension(/g' src/nurbscurve.cpp
sed -i 's/\bcv_size(/get_cv_size(/g' src/nurbscurve.cpp
sed -i 's/\bknot_count(/get_knot_count(/g' src/nurbscurve.cpp
sed -i 's/\bspan_count(/get_span_count(/g' src/nurbscurve.cpp
sed -i 's/\bknot(/get_knot(/g' src/nurbscurve.cpp
sed -i 's/\bknot_multiplicity(/get_knot_multiplicity(/g' src/nurbscurve.cpp
sed -i 's/\bweight(/get_weight(/g' src/nurbscurve.cpp
sed -i 's/\bdomain(/get_domain(/g' src/nurbscurve.cpp
sed -i 's/\bdomain_start(/get_domain_start(/g' src/nurbscurve.cpp
sed -i 's/\bdomain_end(/get_domain_end(/g' src/nurbscurve.cpp
sed -i 's/\bdomain_middle(/get_domain_middle(/g' src/nurbscurve.cpp

echo "Updated internal callers in nurbscurve.cpp"

# Update nurbscurve_test.cpp
sed -i 's/\.degree(/\.get_degree(/g' src/nurbscurve_test.cpp
sed -i 's/\.order(/\.get_order(/g' src/nurbscurve_test.cpp
sed -i 's/\.cv_count(/\.get_cv_count(/g' src/nurbscurve_test.cpp
sed -i 's/\.dimension(/\.get_dimension(/g' src/nurbscurve_test.cpp
sed -i 's/\.cv_size(/\.get_cv_size(/g' src/nurbscurve_test.cpp
sed -i 's/\.knot_count(/\.get_knot_count(/g' src/nurbscurve_test.cpp
sed -i 's/\.span_count(/\.get_span_count(/g' src/nurbscurve_test.cpp
sed -i 's/\.knot(/\.get_knot(/g' src/nurbscurve_test.cpp
sed -i 's/\.knot_multiplicity(/\.get_knot_multiplicity(/g' src/nurbscurve_test.cpp
sed -i 's/\.weight(/\.get_weight(/g' src/nurbscurve_test.cpp
sed -i 's/\.domain(/\.get_domain(/g' src/nurbscurve_test.cpp
sed -i 's/\.domain_start(/\.get_domain_start(/g' src/nurbscurve_test.cpp
sed -i 's/\.domain_end(/\.get_domain_end(/g' src/nurbscurve_test.cpp
sed -i 's/\.domain_middle(/\.get_domain_middle(/g' src/nurbscurve_test.cpp
sed -i 's/\.superfluous_knot(/\.get_superfluous_knot(/g' src/nurbscurve_test.cpp
sed -i 's/\.set_domain(/\.set_domain(/g' src/nurbscurve_test.cpp

echo "Updated nurbscurve_test.cpp"

# Update main_1.cpp
sed -i 's/\.degree(/\.get_degree(/g' main_1.cpp
sed -i 's/\.order(/\.get_order(/g' main_1.cpp
sed -i 's/\.cv_count(/\.get_cv_count(/g' main_1.cpp
sed -i 's/\.knot_count(/\.get_knot_count(/g' main_1.cpp
sed -i 's/\.knot(/\.get_knot(/g' main_1.cpp
sed -i 's/\.domain(/\.get_domain(/g' main_1.cpp

echo "Updated main_1.cpp"

echo "Done!"
