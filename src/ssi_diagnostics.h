#pragma once
#include <array>
#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

namespace session_cpp { class NurbsSurface; }

namespace session_cpp::ssi_diagnostics {

using Parameters = std::array<double, 4>;
using SeedPair = std::array<Parameters, 2>;

std::string boundary_point_image_diagnostic(const NurbsSurface& a, const NurbsSurface& b,
    const std::array<Parameters, 3>& source, const std::array<Parameters, 3>& destination,
    const std::array<int, 2>& fixed, double tolerance);

std::string loop_seam_diagnostic(const NurbsSurface& a, const NurbsSurface& b,
    const std::vector<Parameters>& input, double tolerance);

std::string seam_step_diagnostic(const NurbsSurface& a, const NurbsSurface& b,
    const Parameters& first, const Parameters& last, double tolerance);

bool raw_graph_contains(const Parameters& lower, const Parameters& upper,
    const Parameters& root_lower, const Parameters& root_upper);

std::string seed_chart_diagnostic(const NurbsSurface& a, const NurbsSurface& b,
    const Parameters& lower, const Parameters& upper, const Parameters& root, double tolerance);

enum class Stage {
    Total, Initialize, Discovery, Bounds, Separation, Root, Coverage, Subdivision,
    Trace, TraceConsumption, DensifyAndSeams, Fit, Interpolation, Refinement,
    Certificate, Continuation, SectionConsumption, Count
};

inline constexpr std::array<const char*, static_cast<std::size_t>(Stage::Count)> names = {
    "total", "initialize", "discovery", "bounds", "separation", "root", "coverage", "subdivision",
    "trace", "trace_consumption", "densify_and_seams", "fit", "interpolation", "refinement",
    "certificate", "continuation", "section_consumption"
};

struct Statistics {
    std::array<double, names.size()> seconds{};
    std::size_t cells = 0, box_rejections = 0, projection_rejections = 0;
    std::size_t root_searches = 0, corrections = 0, roots_found = 0;
    std::size_t coverage_checks = 0, covered = 0, seed_comparisons = 0;
    std::size_t traces = 0, pieces = 0, fit_iterations = 0;
    std::size_t continuation_arcs = 0, local_projections = 0;
    std::size_t junction_polishes = 0, polished_junctions = 0;
    std::size_t continuation_failures = 0, continuation_diagnostic_calls = 0;
    int continuation_max_depth = -1, continuation_diagnostic_depth = -8;
    Parameters continuation_first{}, continuation_last{}, continuation_middle{};
    std::string continuation_failure_stage, continuation_diagnostic;
    int max_depth = 0;
    std::vector<Parameters> accepted_seeds, trace_starts;
    std::vector<SeedPair> duplicate_seeds, consumed_seeds;
    std::vector<std::size_t> fit_sizes;
};

using ProgressCallback = void (*)(void*, const Statistics&, Stage, double) noexcept;

struct Progress {
    Statistics* statistics = nullptr;
    ProgressCallback callback = nullptr;
    void* context = nullptr;
    std::chrono::steady_clock::time_point started, reported;
};

inline thread_local Statistics* active = nullptr;
inline thread_local Progress* progress = nullptr;

class Capture {
    Statistics* previous;
    Progress* previous_progress;
    Progress reporting;
public:
    explicit Capture(Statistics& statistics, ProgressCallback callback = nullptr, void* context = nullptr)
        : previous(active), previous_progress(progress) {
        active = &statistics;
        progress = nullptr;
        if (callback) {
            reporting = {&statistics, callback, context};
            reporting.started = reporting.reported = std::chrono::steady_clock::now();
            progress = &reporting;
        }
    }
    ~Capture() { active = previous; progress = previous_progress; }
    Capture(const Capture&) = delete;
    Capture& operator=(const Capture&) = delete;
};

/// Timings include child stages; optional progress uses the same completion timestamp.
class Timer {
    Statistics* statistics;
    Stage stage;
    std::chrono::steady_clock::time_point started;
public:
    explicit Timer(Stage value) : statistics(active), stage(value) {
        if (statistics)
            started = std::chrono::steady_clock::now();
    }
    ~Timer() { stop(); }
    void stop() {
        if (statistics) {
            const auto now = std::chrono::steady_clock::now();
            Statistics* completed = statistics;
            completed->seconds[static_cast<std::size_t>(stage)] +=
                std::chrono::duration<double>(now - started).count();
            statistics = nullptr;
            if (progress && progress->statistics == completed && now - progress->reported >= std::chrono::seconds(5)) {
                progress->reported = now;
                progress->callback(progress->context, *completed, stage,
                    std::chrono::duration<double>(now - progress->started).count());
            }
        }
    }
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;
};

}
