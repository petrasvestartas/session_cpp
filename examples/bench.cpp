#include "session.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <fstream>

namespace session_cpp::bench {

constexpr size_t LARGEST = 1000000; // The largest flat scene; the others are a tenth and a thousandth of it.
constexpr double SCALE = 1.0;       // Slack on bulk and slice budgets for a slower kernel.
constexpr size_t WARMUP = 5;        // Untimed runs before the timed ones.
constexpr size_t RUNS = 101;        // Timed runs; their median is the cost.
constexpr double PER_OBJECT = 0.005; // Milliseconds each object of a bulk step may cost.

/// A session with n points in one flat group, its group and their guids.
struct Flat {
    Session session;                 // The scene.
    std::shared_ptr<TreeNode> group; // The group holding every point.
    std::vector<std::string> guids;  // The point guids in add order.
};

/// Return whether cpu0 runs at its full clock, so absolute budgets mean something.
bool unthrottled() {

    std::string scaling;
    std::string full;
    std::ifstream("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq") >> scaling;
    std::ifstream("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq") >> full;

    return !scaling.empty() && scaling == full;
}

/// Return the milliseconds one call of f takes.
template <typename F> double clock_ms(F&& f) {

    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    f();

    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

/// Return the median of the timings.
double median(std::vector<double> times) {

    std::sort(times.begin(), times.end());

    return times[times.size() / 2];
}

/// Return the largest of the timings.
double largest(const std::vector<double>& times) {
    return times.empty() ? 0.0 : *std::max_element(times.begin(), times.end());
}

/// Print one budget line.
void verdict(const char* name, bool pass) {
    std::printf("%-20s %s\n", name, pass ? "ok" : "OVER BUDGET");
}

/// A session with n points in one flat group.
std::unique_ptr<Flat> flat(size_t n) {

    std::unique_ptr<Flat> scene = std::make_unique<Flat>();
    scene->group = scene->session.add_group("flat");

    for (size_t i = 0; i < n; i++)
        scene->guids.push_back(
            scene->session.add_point(std::make_shared<Point>(static_cast<double>(i), 0.0, 0.0), scene->group)->name
        );

    return scene;
}

/// A quad grid mesh of n by n vertices.
std::shared_ptr<Mesh> grid_mesh(size_t n) {

    std::vector<Point> vertices;
    std::vector<std::vector<size_t>> faces;

    for (size_t at = 0; at < n * n; at++)
        vertices.push_back(Point(static_cast<double>(at / n), static_cast<double>(at % n), 0.0));

    for (size_t cell = 0; cell < (n - 1) * (n - 1); cell++) {
        const size_t at = cell / (n - 1) * n + cell % (n - 1);
        faces.push_back({at, at + n, at + n + 1, at + 1});
    }

    return std::make_shared<Mesh>(Mesh::from_vertices_and_faces(vertices, faces));
}

/// Median of remove, undo, redo, replace, move and add for each flat size: must not grow with the size.
void edit_latency(const std::array<size_t, 3>& sizes) {

    std::vector<std::vector<double>> medians;

    for (size_t n : sizes) {

        std::unique_ptr<Flat> scene = flat(n);
        Session& session = scene->session;
        std::vector<std::vector<double>> times(6);

        for (size_t run = 0; run < WARMUP + RUNS; run++) {

            const std::string& guid = scene->guids[run];
            const std::string& other = scene->guids[n - 1 - run];
            const double x = static_cast<double>(run);
            const std::array<double, 6> lap = {
                clock_ms([&]() {
                    session.begin("remove");
                    session.remove_object(guid);
                    session.commit();
                }),
                clock_ms([&]() {
                    session.undo();
                }),
                clock_ms([&]() {
                    session.redo();
                }),
                clock_ms([&]() {
                    session.begin("replace");
                    session.replace(other, std::make_shared<Point>(x, 1.0, 0.0));
                    session.commit();
                }),
                clock_ms([&]() {
                    session.begin("move");
                    session.set_xform(other, Xform::translation(x, 0.0, 0.0));
                    session.commit();
                }),
                clock_ms([&]() {
                    session.begin("add");
                    session.add_point(std::make_shared<Point>(x, 2.0, 0.0), scene->group);
                    session.commit();
                }),
            };

            if (run < WARMUP)
                continue;

            for (size_t k = 0; k < 6; k++)
                times[k].push_back(lap[k]);
        }

        std::vector<double> row;

        for (const std::vector<double>& laps : times)
            row.push_back(median(laps));

        std::printf(
            "edit latency n=%-8zu remove %.4f undo %.4f redo %.4f replace %.4f move %.4f add %.4f ms\n",
            n, row[0], row[1], row[2], row[3], row[4], row[5]
        );
        medians.push_back(row);
    }

    bool fast = true;
    bool level = true;

    for (const std::vector<double>& row : medians)
        for (double time : row)
            fast = fast && time < 1.0;

    for (size_t k = 0; k < 6; k++)
        level = level && medians[2][k] < 3.0 * medians[0][k] + 0.05;

    verdict("edit latency fast", !unthrottled() || fast);
    verdict("edit latency level", level);
}

/// A bulk remove of bulk objects with its undo and redo: under PER_OBJECT each, level across sizes.
void bulk_undo(size_t n, size_t bulk) {

    std::vector<std::vector<double>> medians;

    for (size_t size : {n, n / 5}) {

        std::unique_ptr<Flat> scene = flat(size);
        Session& session = scene->session;
        std::vector<std::vector<double>> times(3);

        for (size_t run = 0; run < WARMUP + RUNS; run++) {

            const std::array<double, 3> lap = {
                clock_ms([&]() {
                    session.begin("remove");

                    for (size_t i = 0; i < bulk; i++)
                        session.remove_object(scene->guids[i]);

                    session.commit();
                }),
                clock_ms([&]() {
                    session.undo();
                }),
                clock_ms([&]() {
                    session.redo();
                }),
            };
            session.undo();

            if (run < WARMUP)
                continue;

            for (size_t k = 0; k < 3; k++)
                times[k].push_back(lap[k]);
        }

        std::vector<double> row;

        for (const std::vector<double>& laps : times)
            row.push_back(median(laps));

        std::printf(
            "bulk undo n=%-8zu remove %.2f undo %.2f redo %.2f ms for %zu objects\n",
            size, row[0], row[1], row[2], bulk
        );
        medians.push_back(row);
    }

    const double budget = PER_OBJECT * static_cast<double>(bulk) * SCALE;
    bool over = true;
    bool level = true;

    for (size_t k = 0; k < 3; k++) {
        over = over && medians[0][k] < budget;
        level = level && medians[0][k] < 1.5 * medians[1][k];
    }

    std::printf("bulk budget %.1f ms\n", budget);
    verdict("bulk undo budget", !unthrottled() || over);
    verdict("bulk undo level", level);
}

/// Purge and checkpoint slices after a dropped bulk remove: every slice under a frame.
void no_pauses(size_t n, size_t bulk) {

    std::unique_ptr<Flat> scene = flat(n);
    Session& session = scene->session;
    session.begin("remove");

    for (size_t i = 0; i < bulk; i++)
        session.remove_object(scene->guids[i]);

    session.commit();

    for (int step = 0; step < CAPACITY; step++) {
        session.begin("move");
        session.set_xform(scene->guids[n - 1], Xform::translation(static_cast<double>(step), 0.0, 0.0));
        session.commit();
    }

    std::vector<double> purges;
    bool purging = true;

    while (purging)
        purges.push_back(clock_ms([&]() {
            purging = session.purge_step(PURGE_WORK);
        }));

    std::vector<double> writes;
    std::optional<std::string> bytes;

    while (!bytes)
        writes.push_back(clock_ms([&]() {
            bytes = session.checkpoint(PURGE_WORK);
        }));

    const bool sliced = largest(purges) < 16.0 && largest(writes) < 16.0;
    std::printf(
        "no pauses n=%-8zu %zu purge slices median %.3f ms max %.3f ms, %zu write slices max %.3f ms\n",
        n, purges.size(), median(purges), largest(purges), writes.size(), largest(writes)
    );
    verdict("slices under 16 ms", !unthrottled() || sliced);
    verdict("purge slice median", !unthrottled() || median(purges) < 2.0 * SCALE);
}

/// Edit and undo cost after 10k remove/undo/redo/add cycles with idle purging: level with the first hundred.
void steady_state(size_t n) {

    const size_t cycles = std::min<size_t>(10000, n);
    std::unique_ptr<Flat> scene = flat(n);
    Session& session = scene->session;
    std::vector<double> edits;
    std::vector<double> undos;

    for (size_t cycle = 0; cycle < cycles; cycle++) {
        edits.push_back(clock_ms([&]() {
            session.begin("remove");
            session.remove_object(scene->guids[cycle]);
            session.commit();
        }));
        undos.push_back(clock_ms([&]() {
            session.undo();
        }));
        session.redo();
        session.begin("add");
        session.add_point(std::make_shared<Point>(static_cast<double>(cycle), 1.0, 0.0), scene->group);
        session.commit();
        session.purge_step(PURGE_WORK);
    }

    const std::vector<double> early_edits(edits.begin(), edits.begin() + 100);
    const std::vector<double> late_edits(edits.end() - 100, edits.end());
    const std::vector<double> early_undos(undos.begin(), undos.begin() + 100);
    const std::vector<double> late_undos(undos.end() - 100, undos.end());
    std::printf(
        "steady state n=%-8zu edit %.4f -> %.4f ms, undo %.4f -> %.4f ms, %zu dead, %zu bytes\n",
        n, median(early_edits), median(late_edits), median(early_undos), median(late_undos),
        session.number_of_dead(), session.history.bytes
    );
    verdict("steady edit level", median(late_edits) <= 1.5 * median(early_edits) + 0.05);
    verdict("steady undo level", median(late_undos) <= 1.5 * median(early_undos) + 0.05);
}

/// Commit cost of 200 mesh removes under an 8 MiB budget: level once the budget evicts.
void history_memory() {

    Session session;
    session.history.budget = 8 << 20;
    std::vector<std::string> guids;

    for (int i = 0; i < 200; i++) {
        const std::shared_ptr<Mesh> mesh = grid_mesh(100);
        guids.push_back(mesh->guid());
        session.add_mesh(mesh);
    }

    std::vector<double> commits;

    for (const std::string& guid : guids)
        commits.push_back(clock_ms([&]() {
            session.begin("remove");
            session.remove_object(guid);
            session.commit();
        }));

    const std::vector<double> early(commits.begin(), commits.begin() + 50);
    const std::vector<double> late(commits.begin() + 150, commits.end());
    std::printf(
        "history memory     commit %.4f -> %.4f ms, depth %d, %zu bytes\n",
        median(early), median(late), session.history.depth(), session.history.bytes
    );
    verdict("history commit level", median(late) <= 2.0 * median(early));
}

/// Add and remove with a transaction open against the same unrecorded: the record must cost nothing visible.
void record_cost(size_t n) {

    std::unique_ptr<Flat> scene = flat(n);
    Session& session = scene->session;
    std::array<std::vector<double>, 2> plain;
    std::array<std::vector<double>, 2> recorded;

    for (size_t run = 0; run < WARMUP + RUNS; run++) {

        const double x = static_cast<double>(run);
        const double add = clock_ms([&]() {
            session.add_point(std::make_shared<Point>(x, 1.0, 0.0), scene->group);
        });
        const double remove = clock_ms([&]() {
            session.remove_object(scene->guids[run]);
        });
        session.begin("record");
        const double add_recorded = clock_ms([&]() {
            session.add_point(std::make_shared<Point>(x, 2.0, 0.0), scene->group);
        });
        const double remove_recorded = clock_ms([&]() {
            session.remove_object(scene->guids[WARMUP + RUNS + run]);
        });
        session.commit();

        if (run < WARMUP)
            continue;

        plain[0].push_back(add);
        plain[1].push_back(remove);
        recorded[0].push_back(add_recorded);
        recorded[1].push_back(remove_recorded);
    }

    std::printf(
        "record cost n=%-8zu add %.4f vs %.4f ms, remove %.4f vs %.4f ms\n",
        n, median(plain[0]), median(recorded[0]), median(plain[1]), median(recorded[1])
    );
    bool cheap = true;

    for (size_t k = 0; k < 2; k++)
        cheap = cheap && median(recorded[k]) - median(plain[k]) < 0.02 * SCALE;

    verdict("record cost", !unthrottled() || cheap);
}

/// Moving 10k nodes between two groups with 1k then 100k unrelated objects: the cost must not follow the unrelated count.
void layer_move(const std::array<size_t, 2>& unrelated_sizes) {

    std::vector<std::vector<double>> medians;

    for (size_t unrelated : unrelated_sizes) {

        Session session;
        const std::shared_ptr<TreeNode> a = session.add_group("a");
        const std::shared_ptr<TreeNode> b = session.add_group("b");
        const std::shared_ptr<TreeNode> elsewhere = session.add_group("elsewhere");
        std::vector<std::shared_ptr<TreeNode>> nodes;
        std::vector<std::vector<double>> times(3);

        for (int i = 0; i < 10000; i++)
            nodes.push_back(session.add_point(std::make_shared<Point>(static_cast<double>(i), 0.0, 0.0), a));

        for (size_t i = 0; i < unrelated; i++)
            session.add_point(std::make_shared<Point>(static_cast<double>(i), 1.0, 0.0), elsewhere);

        for (size_t run = 0; run < WARMUP + RUNS; run++) {

            const std::shared_ptr<TreeNode>& target = run % 2 == 0 ? b : a;
            const std::array<double, 3> lap = {
                clock_ms([&]() {
                    session.begin("move");

                    for (const std::shared_ptr<TreeNode>& node : nodes)
                        session.add(node, target);

                    session.commit();
                }),
                clock_ms([&]() {
                    session.undo();
                }),
                clock_ms([&]() {
                    session.redo();
                }),
            };

            while (session.purge_step(PURGE_WORK)) {}

            if (run < WARMUP)
                continue;

            for (size_t k = 0; k < 3; k++)
                times[k].push_back(lap[k]);
        }

        std::vector<double> row;

        for (const std::vector<double>& laps : times)
            row.push_back(median(laps));

        std::printf(
            "layer move unrelated=%-7zu move %.2f undo %.2f redo %.2f ms\n", unrelated, row[0], row[1], row[2]
        );
        medians.push_back(row);
    }

    bool bulk = true;
    bool level = true;

    for (size_t k = 0; k < 3; k++) {
        bulk = bulk && medians[1][k] < 50.0 * SCALE;
        level = level && medians[1][k] < 3.0 * medians[0][k];
    }

    verdict("layer move budget", !unthrottled() || bulk);
    verdict("layer move level", level);
}

} // namespace session_cpp::bench

int main() {

    using namespace session_cpp::bench;
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::array<size_t, 3> sizes = {LARGEST / 1000, LARGEST / 10, LARGEST};
    const size_t bulk = LARGEST / 10;
    std::printf(
        "sizes [%zu, %zu, %zu], bulk %zu, cpu %s\n",
        sizes[0], sizes[1], sizes[2], bulk, unthrottled() ? "unthrottled" : "throttled"
    );
    edit_latency(sizes);
    bulk_undo(LARGEST, bulk);
    no_pauses(LARGEST, bulk);
    steady_state(sizes[1]);
    history_memory();
    record_cost(sizes[1]);
    layer_move({sizes[0], sizes[1]});

    return 0;
}

// Undo/redo cost of the tombstone kernel, printed, never asserted: cmake --build build --target session_bench && ./build/session_bench
