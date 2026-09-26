#include "mini_test.h"
#include "collection.h"
#include "file_encoders.h"
#include "history.h"
#include "objects.h"
#include "point.h"
#include "tolerance.h"
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Collection", "Constructor") {
    // using session_cpp::Collection;
    // using session_cpp::Point;

    const std::shared_ptr<Point> a = std::make_shared<Point>(0.0, 0.0, 0.0);
    const std::shared_ptr<Point> b = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::shared_ptr<Point> c = std::make_shared<Point>(2.0, 0.0, 0.0);
    Collection<std::shared_ptr<Point>> points;
    points.push_back(a);
    points.push_back(b);
    points.push_back(c);
    const std::vector<std::shared_ptr<Point>> iterated(points.begin(), points.end());

    MINI_CHECK(points.size() == 3);
    MINI_CHECK(points[0] == a && points[1] == b);
    MINI_CHECK(points[2] == c);
    MINI_CHECK(iterated[0] == a && iterated[2] == c);
    MINI_CHECK(points.get_slot(b->guid()) == std::optional<size_t>(1));
    MINI_CHECK(points.number_of_slots() == 3);
    MINI_CHECK(points.number_of_dead() == 0);
    MINI_CHECK(points.str() == "Collection(3 live, 0 dead)");
}

MINI_TEST("Collection", "Set Dead") {
    // using session_cpp::Collection;
    // using session_cpp::Point;

    const std::shared_ptr<Point> a = std::make_shared<Point>(0.0, 0.0, 0.0);
    const std::shared_ptr<Point> b = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::shared_ptr<Point> c = std::make_shared<Point>(2.0, 0.0, 0.0);
    Collection<std::shared_ptr<Point>> points(std::vector<std::shared_ptr<Point>>{a, b, c});
    points.set_dead(1, true);
    const std::vector<std::shared_ptr<Point>> killed = points.to_vector();
    const size_t killed_len = points.size();
    const std::optional<size_t> killed_slot = points.get_slot(b->guid());
    const size_t killed_dead = points.number_of_dead();
    const size_t killed_slots = points.number_of_slots();
    const bool kept = points.get_item(1) == b && points.is_dead(1);
    const bool second = points[1] == c;
    points.set_dead(1, false);

    MINI_CHECK(killed_len == 2 && second);
    MINI_CHECK(killed[0] == a && killed[1] == c);
    MINI_CHECK(!killed_slot && kept);
    MINI_CHECK(killed_dead == 1 && killed_slots == 3);
    MINI_CHECK(points.size() == 3);
    MINI_CHECK(points[1] == b && points[2] == c);
    MINI_CHECK(points.get_slot(b->guid()) == std::optional<size_t>(1));
}

MINI_TEST("Collection", "Index Skips Dead") {
    // using session_cpp::Collection;
    // using session_cpp::Point;

    std::vector<std::shared_ptr<Point>> e;

    for (int i = 0; i < 6; ++i)
        e.push_back(std::make_shared<Point>(static_cast<double>(i), 0.0, 0.0));

    Collection<std::shared_ptr<Point>> points(std::vector<std::shared_ptr<Point>>(e.begin(), e.begin() + 5));
    points.set_dead(0, true);
    points.set_dead(3, true);
    bool missing = false;

    try {
        points.at(3);
    } catch (const std::out_of_range&) {
        missing = true;
    }

    points.push_back(e[5]);

    MINI_CHECK(points[0] == e[1] && points[1] == e[2]);
    MINI_CHECK(points[2] == e[4]);
    MINI_CHECK(points.front() == e[1]);
    MINI_CHECK(missing);
    MINI_CHECK(points[3] == e[5]);
    MINI_CHECK(points.back() == e[5]);
}

MINI_TEST("Collection", "Compact") {
    // using session_cpp::Collection;
    // using session_cpp::Point;
    // using session_cpp::Tomb;

    std::vector<std::shared_ptr<Point>> e;

    for (int i = 0; i < 5; ++i)
        e.push_back(std::make_shared<Point>(static_cast<double>(i), 0.0, 0.0));

    Collection<std::shared_ptr<Point>> points(e);
    std::shared_ptr<Tomb> tomb = std::make_shared<Tomb>("points", false, 3, nullptr);
    points.set_dead(1, true);
    points.set_dead(3, true);
    points.set_tomb(3, tomb);
    points.compact();
    const size_t pinned_slots = points.number_of_slots();
    const size_t pinned_dead = points.number_of_dead();
    const std::vector<std::shared_ptr<Point>> order = points.to_vector();
    const std::vector<std::optional<size_t>> slots = {
        points.get_slot(e[0]->guid()),
        points.get_slot(e[2]->guid()),
        points.get_slot(e[4]->guid()),
    };
    const size_t moved = tomb->slot;
    tomb.reset();
    points.compact();

    MINI_CHECK(pinned_slots == 4 && pinned_dead == 1 && moved == 2);
    MINI_CHECK(order[0] == e[0] && order[1] == e[2]);
    MINI_CHECK(order[2] == e[4]);
    MINI_CHECK((slots == std::vector<std::optional<size_t>>{0, 1, 3}));
    MINI_CHECK(points.number_of_slots() == 3 && points.number_of_dead() == 0);
    MINI_CHECK(points.get_slot(e[4]->guid()) == std::optional<size_t>(2));
}

MINI_TEST("Collection", "Compact Step") {
    // using session_cpp::Collection;
    // using session_cpp::Point;
    // using session_cpp::Tomb;

    Collection<std::shared_ptr<Point>> points;
    std::vector<std::pair<std::shared_ptr<Point>, bool>> model;
    std::vector<std::shared_ptr<Tomb>> tombs;

    for (int i = 0; i < 1000; ++i) {

        const std::shared_ptr<Point> point = std::make_shared<Point>(static_cast<double>(i), 0.0, 0.0);
        points.push_back(point);
        model.emplace_back(point, i % 3 != 0);
    }

    for (size_t i = 0; i < 1000; i += 3) {

        points.set_dead(i, true);

        if (tombs.size() < 10) {
            tombs.push_back(std::make_shared<Tomb>("points", false, i, nullptr));
            points.set_tomb(i, tombs.back());
        }
    }

    bool bounded = true;
    bool exact = true;
    size_t revived = 0;

    while (true) {

        bounded = bounded && points.compact_step(10) <= 10;
        std::vector<std::shared_ptr<Point>> expected;

        for (const std::pair<std::shared_ptr<Point>, bool>& m : model)
            if (m.second)
                expected.push_back(m.first);

        exact = exact && points.to_vector() == expected;

        for (const std::shared_ptr<Point>& p : points) {
            const std::optional<size_t> s = points.get_slot(p->guid());
            exact = exact && s && points.get_item(*s) == p;
        }

        if (!points.is_compacting())
            break;

        const std::shared_ptr<Point> point = std::make_shared<Point>(-1.0, 0.0, 0.0);
        points.push_back(point);
        model.emplace_back(point, true);

        if (revived < 5) {
            points.set_dead(tombs[revived]->slot, false);
            model[revived * 3].second = true;
            revived++;
        }
    }

    const bool settled = points.number_of_slots() == points.size() + 5;
    points.compact_step(10);
    points.set_dead(1, true);

    while (points.is_compacting())
        points.compact_step(10);

    const bool waiting = points.is_dead(1) && points.number_of_dead() == 6;
    points.compact();

    MINI_CHECK(bounded && exact && settled);
    MINI_CHECK(waiting);
    MINI_CHECK(points.number_of_dead() == 5);
    MINI_CHECK(points.number_of_slots() == points.size() + 5);
}

MINI_TEST("Collection", "Json Roundtrip") {
    // using session_cpp::Point;
    // using session_cpp::Objects;

    Objects objects;
    objects.points->push_back(std::make_shared<Point>(0.0, 0.0, 0.0));
    objects.points->push_back(std::make_shared<Point>(1.0, 0.0, 0.0));
    objects.points->push_back(std::make_shared<Point>(2.0, 0.0, 0.0));
    objects.points->set_dead(1, true);
    const nlohmann::ordered_json json = objects.jsondump()["points"];
    const Objects loaded = Objects::jsonload(objects.jsondump());

    MINI_CHECK(json == file_encoders::file_encode_collection(objects.points->to_vector()));
    MINI_CHECK(loaded.points->size() == 2 && loaded.points->number_of_dead() == 0);
    MINI_CHECK((*loaded.points)[0]->guid() == (*objects.points)[0]->guid());
    MINI_CHECK((*loaded.points)[1]->guid() == (*objects.points)[1]->guid());
}

} // namespace session_cpp
