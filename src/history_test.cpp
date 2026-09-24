#include "mini_test.h"
#include "history.h"
#include "session.h"
#include "point.h"
#include "tolerance.h"

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("History", "Constructor") {

    History history;
    const std::string hstr = history.str();
    const std::string hrepr = history.repr();

    MINI_CHECK(!history.can_undo());
    MINI_CHECK(!history.can_redo());
    MINI_CHECK(history.depth() == 0);
    MINI_CHECK(hstr == "History(0 undo, 0 redo)");
    MINI_CHECK(hrepr == "History(0 undo, 0 redo)");
}

MINI_TEST("History", "Begin Commit") {

    Session session;
    History& history = session.history;

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
    MINI_CHECK(std::get<AddOp>(history.undo_stack[0].ops[0]).kind == "add");
}

MINI_TEST("History", "Undo Redo") {

    Session session;
    const std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    const std::string guid = point->guid();

    session.history.begin("add");
    session.add_point(point);
    session.history.commit();

    const bool undone = session.history.undo(session);
    const bool absent = session.lookup.count(guid) == 0;
    const bool redone = session.history.redo(session);

    MINI_CHECK(undone);
    MINI_CHECK(absent);
    MINI_CHECK(redone);
    MINI_CHECK(session.lookup.count(guid) == 1);
    MINI_CHECK(TOLERANCE.is_close((*std::get<std::shared_ptr<Point>>(session.lookup[guid]))[2], 3.0));
    MINI_CHECK(!session.history.can_redo());
    MINI_CHECK(!session.history.redo(session));
}

MINI_TEST("History", "Clear") {

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

MINI_TEST("History", "Undo Definition") {

    Session session;
    const std::shared_ptr<Point> point = std::make_shared<Point>(1.0, 2.0, 3.0);
    const std::string guid = point->guid();

    session.begin("define");
    session.add_definition(point);
    session.commit();

    session.begin("replace");
    session.replace_definition(guid, std::make_shared<Point>(9.0, 9.0, 9.0));
    session.commit();

    session.begin("remove");
    session.remove_definition(guid);
    session.commit();

    const bool removed = session.definition_lookup.count(guid) == 0;
    session.undo();
    const double replaced = (*std::get<std::shared_ptr<Point>>(session.definition_lookup[guid]))[0];
    session.undo();
    const double restored = (*std::get<std::shared_ptr<Point>>(session.definition_lookup[guid]))[0];
    session.undo();
    const bool undefined = session.definition_lookup.empty() && session.definitions.points->empty();
    const bool redone = session.redo();

    MINI_CHECK(removed);
    MINI_CHECK(TOLERANCE.is_close(replaced, 9.0));
    MINI_CHECK(TOLERANCE.is_close(restored, 1.0));
    MINI_CHECK(undefined);
    MINI_CHECK(redone);
    MINI_CHECK(session.definitions.points->size() == 1);
    MINI_CHECK(session.definitions.points->at(0)->guid() == guid);
}

} // namespace session_cpp
