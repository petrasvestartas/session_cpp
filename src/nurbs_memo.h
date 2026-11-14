#pragma once

#include "curve_intersection.h"
#include "nurbssurface.h"
#include "surface_ray.h"
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace session_cpp::nurbs_memo {

/// A copy of a surface and its recognised form, which refers to the copy.
struct Recognised {
    NurbsSurface surface;
    std::optional<surface_ray::Form> form;
};

/// Complete results of curve_intersection::intersect and matched same_parameter certificates, reused on this thread while the scope lives. A key is the bytes of the whole input, so an entry can not go stale; failures are never stored.
struct Scope {
    std::unordered_map<std::string, curve_intersection::Result> curve_pairs;
    std::unordered_set<std::string> matched;
    /// By surface, then by one span of a curve and one of its pcurve over the same raw range: the bound a matched certificate reached there.
    std::unordered_map<std::string, std::unordered_map<std::string, double>> spans;
    /// By surface and the tolerance it was recognised at.
    std::unordered_map<std::string, std::unique_ptr<Recognised>> forms;
    Scope* outer = active;
    Scope() { active = this; }
    ~Scope() { active = outer; }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
    static inline thread_local Scope* active = nullptr;
};

template <typename T>
void add(std::string& key, const T* values, size_t count) {
    key.append(reinterpret_cast<const char*>(&count), sizeof count);
    key.append(reinterpret_cast<const char*>(values), count * sizeof(T));
}

inline void add(std::string& key, const NurbsCurve& curve) {
    const int layout[] = {curve.m_dim, curve.m_is_rat, curve.m_order, curve.m_cv_count, curve.m_cv_stride};
    add(key, layout, std::size(layout));
    add(key, curve.m_nurbsknot.data(), curve.m_nurbsknot.size());
    add(key, curve.m_cv.data(), curve.m_cv.size());
}

inline void add(std::string& key, const NurbsSurface& surface) {
    const int layout[] = {surface.m_dim, surface.m_is_rat, surface.m_order[0], surface.m_order[1],
        surface.m_cv_count[0], surface.m_cv_count[1], surface.m_cv_stride[0], surface.m_cv_stride[1]};
    add(key, layout, std::size(layout));
    for (const std::vector<double>& knots : surface.m_nurbsknot)
        add(key, knots.data(), knots.size());
    add(key, surface.m_cv.data(), surface.m_cv.size());
}

/// The bytes of the polynomial piece of a curve on the span around a parameter: its controls and the knots they blend over.
inline void add_span(std::string& key, const NurbsCurve& curve, double inside) {
    const int first = nurbsknot::find_span(curve.order(), curve.cv_count(), curve.m_nurbsknot, inside);
    const int layout[] = {curve.m_dim, curve.m_is_rat, curve.m_order};
    add(key, layout, std::size(layout));
    add(key, &curve.m_nurbsknot[first], 2 * curve.degree());
    for (int i = 0; i < curve.order(); ++i)
        add(key, curve.cv(first + i), curve.cv_size());
}

/// A surface kept while the scope lives, by its bytes and the tolerance its form is recognised at, for the form to be laid out on once something needs it: null outside a scope.
inline Recognised* recognised(const NurbsSurface& surface, double tolerance) {
    if (!Scope::active)
        return nullptr;
    std::string key;
    add(key, surface);
    add(key, &tolerance, 1);
    std::unique_ptr<Recognised>& entry = Scope::active->forms[key];
    if (!entry)
        entry = std::make_unique<Recognised>(Recognised{surface, std::nullopt});
    return entry.get();
}

}
