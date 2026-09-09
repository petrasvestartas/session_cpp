#include "mini_test.h"
#include "history.h"
#include "session.h"
#include "point.h"
#include "tolerance.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("History", "Constructor") {
    // uncomment #include "history.h"

    History history;
    std::string hstr = history.str();
    std::string hrepr = history.repr();

    MINI_CHECK(!history.can_undo());
    MINI_CHECK(!history.can_redo());
    MINI_CHECK(history.depth() == 0);
    MINI_CHECK(hstr == "History(0 undo, 0 redo)");
    MINI_CHECK(hrepr == "History(0 undo, 0 redo)");
}

MINI_TEST("History", "Begin Commit") {
    // uncomment #include "history.h"
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    History& history = session.history;

    // An empty transaction is dropped, and nothing is recorded while none is open.
    history.begin("empty");
    history.commit();
    session.add_point(std::make_shared<Point>(0.0, 0.0, 0.0));

    history.begin("add");
    session.add_point(std::make_shared<Point>(1.0, 0.0, 0.0));
    history.commit();

    MINI_CHECK(history.depth() == 1);
    MINI_CHECK(history.can_undo());
    MINI_CHECK(history.undo_stack[0].ops.size() == 1);
    MINI_CHECK(history.undo_stack[0].label == "add");
    MINI_CHECK(std::visit([](const auto& op) { return op.kind; }, history.undo_stack[0].ops[0]) == "add");
}

MINI_TEST("History", "Undo Redo") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;
    auto point = std::make_shared<Point>(1.0, 2.0, 3.0);
    std::string guid = point->guid();

    session.history.begin("add");
    session.add_point(point);
    session.history.commit();
    bool undone = session.history.undo(session);
    bool absent = session.lookup.count(guid) == 0;
    bool redone = session.history.redo(session);

    MINI_CHECK(undone);
    MINI_CHECK(absent);
    MINI_CHECK(redone);
    MINI_CHECK(session.lookup.count(guid) == 1);
    MINI_CHECK(TOLERANCE.is_close((*std::get<std::shared_ptr<Point>>(session.lookup[guid]))[2], 3.0));
    MINI_CHECK(!session.history.can_redo());
    MINI_CHECK(!session.history.redo(session));
}

MINI_TEST("History", "Clear") {
    // uncomment #include "session.h"
    // uncomment #include "point.h"

    Session session;

    session.history.begin("a");
    session.add_point(std::make_shared<Point>(0.0, 0.0, 0.0));
    session.history.commit();
    session.history.begin("b");
    session.add_point(std::make_shared<Point>(1.0, 0.0, 0.0));
    session.history.commit();
    session.history.undo(session);
    session.history.clear();

    MINI_CHECK(!session.history.can_undo());
    MINI_CHECK(!session.history.can_redo());
    MINI_CHECK(session.history.depth() == 0);
    MINI_CHECK(session.objects.points->size() == 1);
}

} // namespace session_cpp
