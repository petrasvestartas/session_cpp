#pragma once
#include <array>
#include <cstddef>
#include <chrono>
#include <functional>

namespace session_cpp::native_tessellate_diagnostics {
struct State {
    const char* phase = "call";
    const char* previous_phase = "none";
    const char* metadata_failure = "";
    int pass = -1, face = -1;
    size_t triangles = 0, visited = 0, certified = 0, sampled = 0, measured_passed = 0;
    std::array<std::array<double, 2>, 3> uv{};
    double elapsed_previous_phase_seconds = 0;
    bool checkpoint = false;
    double elapsed_current_phase_seconds = 0;
};
inline thread_local const std::function<void(const State&)>* observer = nullptr;
struct Capture {
    const std::function<void(const State&)>* previous;
    explicit Capture(const std::function<void(const State&)>& callback) : previous(observer) {
        observer = &callback;
    }
    ~Capture() { observer = previous; }
    Capture(const Capture&) = delete;
    Capture& operator=(const Capture&) = delete;
};
struct Progress {
    State state;
    const char* last_phase = "none";
    std::chrono::steady_clock::time_point start, last_checkpoint;
    void phase(const char* name) {
        if (!observer)
            return;
        const auto now = std::chrono::steady_clock::now();
        state.elapsed_previous_phase_seconds = state.pass < 0 ? 0 : std::chrono::duration<double>(now - start).count();
        state.previous_phase = last_phase;
        state.phase = name;
        last_phase = name;
        (*observer)(state);
        start = last_checkpoint = now;
    }
    // Called only at a coarse iteration cadence; no background thread or per-triangle IO.
    void checkpoint() {
        if (!observer)
            return;
        const auto now = std::chrono::steady_clock::now();
        if (state.visited != 1 && now - last_checkpoint < std::chrono::seconds(2))
            return;
        State snapshot = state;
        snapshot.checkpoint = true;
        snapshot.elapsed_current_phase_seconds = std::chrono::duration<double>(now - start).count();
        (*observer)(snapshot);
        last_checkpoint = now;
    }
};
}
