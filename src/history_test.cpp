#include "mini_test.h"
#include "history.h"
#include "session.h"
#include "mesh.h"
#include "point.h"
#include "tolerance.h"
#include <algorithm>

namespace session_cpp {
using namespace session_cpp::mini_test;

/// A mesh of n by n vertices in quads, one guid of its own.
static std::shared_ptr<Mesh> grid_mesh(size_t n) {

    std::vector<Point> vertices;
    std::vector<std::vector<size_t>> faces;
    vertices.reserve(n * n);
    faces.reserve((n - 1) * (n - 1));

    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j)
            vertices.emplace_back(static_cast<double>(i), static_cast<double>(j), 0.0);

    for (size_t i = 0; i + 1 < n; ++i)
        for (size_t j = 0; j + 1 < n; ++j) {
            const size_t at = i * n + j;
            faces.push_back({at, at + n, at + n + 1, at + 1});
        }

    return std::make_shared<Mesh>(Mesh::from_vertices_and_faces(vertices, faces));
}

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

    session.undo();
    session.history.clear();

    MINI_CHECK(!session.history.can_undo());
    MINI_CHECK(!session.history.can_redo());
    MINI_CHECK(session.history.depth() == 0);
    MINI_CHECK(session.history.bytes == 0);
    MINI_CHECK(session.history.dropped == 2);
    MINI_CHECK(session.objects.points->size() == 1);
    MINI_CHECK(session.objects.points->number_of_dead() == 1);
    MINI_CHECK(session.objects.points->number_of_slots() == 2);
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

    const std::string defined = std::get<AddOp>(session.history.undo_stack[0].ops[0]).repr();
    const std::string swapped = std::get<ReplaceOp>(session.history.undo_stack[1].ops[0]).kind;
    const std::string dropped = std::get<RemoveOp>(session.history.undo_stack[2].ops[0]).repr();
    const bool removed = session.definition_lookup.count(guid) == 0;
    session.undo();
    const double replaced = (*std::get<std::shared_ptr<Point>>(session.definition_lookup[guid]))[0];
    session.undo();
    const double restored = (*std::get<std::shared_ptr<Point>>(session.definition_lookup[guid]))[0];
    session.undo();
    const bool undefined = session.definition_lookup.empty() && session.definitions.points->empty();
    const bool redone = session.redo();

    MINI_CHECK(defined == fmt::format("add({}, definitions)", guid));
    MINI_CHECK(swapped == "replace");
    MINI_CHECK(dropped == fmt::format("remove({}, definitions)", guid));
    MINI_CHECK(removed);
    MINI_CHECK(TOLERANCE.is_close(replaced, 9.0));
    MINI_CHECK(TOLERANCE.is_close(restored, 1.0));
    MINI_CHECK(undefined);
    MINI_CHECK(redone);
    MINI_CHECK(session.definitions.points->size() == 1);
    MINI_CHECK(session.definitions.points->number_of_slots() == 1);
    MINI_CHECK(session.definitions.points->at(0)->guid() == guid);
}

MINI_TEST("History", "Budget") {

    Session session;
    session.history.budget = 1 << 20;
    std::vector<std::string> guids;

    for (int i = 0; i < 20; ++i) {
        const std::shared_ptr<Mesh> mesh = grid_mesh(100);
        guids.push_back(mesh->guid());
        session.add_mesh(mesh);
    }

    for (const std::string& guid : guids) {
        session.begin("remove");
        session.remove_object(guid);
        session.commit();
    }

    const size_t newest = session.history.undo_stack[session.history.depth() - 1].bytes;
    size_t pinned = 0;

    for (const Transaction& transaction : session.history.undo_stack)
        pinned += transaction.bytes;

    for (const Transaction& transaction : session.history.redo_stack)
        pinned += transaction.bytes;

    MINI_CHECK(session.history.depth() < 20);
    MINI_CHECK(session.history.bytes <= session.history.budget + newest);
    MINI_CHECK(session.history.dropped > 0);
    MINI_CHECK(session.history.bytes == pinned);
    MINI_CHECK(session.undo());
    MINI_CHECK(session.lookup.count(guids[19]) == 1);
}

MINI_TEST("History", "Weight") {

    const Item point = Geometry(std::make_shared<Point>(0.0, 0.0, 0.0));
    const Item small = Geometry(grid_mesh(32));
    const std::shared_ptr<Mesh> mesh = grid_mesh(100);
    const Item large = Geometry(mesh);
    Session session;
    const std::string guid = mesh->guid();
    const std::shared_ptr<Point> a = std::make_shared<Point>(0.0, 0.0, 0.0);
    const std::shared_ptr<Point> b = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string a_guid = a->guid();
    const std::string b_guid = b->guid();
    session.add_mesh(mesh);
    session.add_point(a);
    session.add_point(b);
    session.add_edge(a_guid, guid, "touch");
    session.add_edge(b_guid, guid, "touch");

    session.begin("remove");
    session.remove_object(guid);
    const size_t removed = session.history.current->bytes;
    session.add_point(std::make_shared<Point>(2.0, 0.0, 0.0));
    const size_t added = session.history.current->bytes;
    session.commit();

    MINI_CHECK(weight(point) < weight(small));
    MINI_CHECK(weight(small) < weight(large));
    MINI_CHECK(removed == RECORD + weight(large) + 128 * 2);
    MINI_CHECK(added == removed + RECORD);
}

MINI_TEST("History", "Abort") {

    Session session;
    const std::shared_ptr<TreeNode> group = session.add_group("g");
    const std::shared_ptr<Point> b = std::make_shared<Point>(1.0, 0.0, 0.0);
    const std::string b_guid = b->guid();
    session.add_point(std::make_shared<Point>(0.0, 0.0, 0.0), group);
    const std::shared_ptr<TreeNode> b_node = session.add_point(b, group);
    session.add_point(std::make_shared<Point>(2.0, 0.0, 0.0), group);
    session.begin("kept");
    session.set_xform(b_guid, Xform::translation(0.0, 1.0, 0.0));
    session.commit();
    session.undo();
    const std::shared_ptr<Point> a = std::make_shared<Point>(5.0, 0.0, 0.0);
    const std::string a_guid = a->guid();

    session.begin("aborted");
    session.add_point(a, group);
    session.remove_object(b_guid);
    const bool aborted = session.abort();
    const std::vector<std::string> order = session.order();

    MINI_CHECK(aborted);
    MINI_CHECK(session.lookup.count(a_guid) == 0);
    MINI_CHECK(std::find(order.begin(), order.end(), a_guid) == order.end());
    MINI_CHECK(session.get_node(a_guid) == nullptr);
    MINI_CHECK(session.lookup.count(b_guid) == 1);
    MINI_CHECK(session.objects.points->get_slot(b_guid) == 1);
    MINI_CHECK(group->children()[1] == b_node.get());
    MINI_CHECK(b_node->at() == 1);
    MINI_CHECK(session.history.depth() == 0);
    MINI_CHECK(session.history.can_redo());
    MINI_CHECK(session.history.redo_stack.size() == 1);
    MINI_CHECK(!session.abort());
}

} // namespace session_cpp
