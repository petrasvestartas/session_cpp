#include "history.h"
#include "session.h"
#include "fmt/core.h"

namespace session_cpp {

Item clone(const Item& obj) {

    if (const Geometry* geometry = std::get_if<Geometry>(&obj))
        return clone(*geometry);

    if (const Component* component = std::get_if<Component>(&obj)) {
        Component snapshot = *component;
        snapshot.guid() = component->guid();
        return snapshot;
    }

    const std::shared_ptr<InstanceRef>& live = std::get<std::shared_ptr<InstanceRef>>(obj);
    std::shared_ptr<InstanceRef> snapshot = std::make_shared<InstanceRef>(*live);
    snapshot->guid() = live->guid();
    snapshot->features = clone(live->features);

    return snapshot;
}

/// A deep copy of one geometry that keeps its guid.
template <typename T>
static Geometry snapshot(const std::shared_ptr<T>& live) {

    std::shared_ptr<T> copy = std::make_shared<T>(*live);
    copy->guid() = live->guid();

    return copy;
}

/// A deep copy of one element that keeps its guid and its feature guids.
static Geometry snapshot(const std::shared_ptr<Element>& live) {

    std::vector<ElementFeature> features = clone(live->features());
    std::shared_ptr<Element> copy = live->clone();
    copy->set_features(std::move(features));
    copy->guid() = live->guid();

    return copy;
}

Geometry clone(const Geometry& obj) {

    return std::visit(
        [](const auto& live) {
            return snapshot(live);
        },
        obj
    );
}

std::vector<ElementFeature> clone(const std::vector<ElementFeature>& features) {

    std::vector<ElementFeature> out(features);

    for (size_t i = 0; i < out.size(); ++i)
        if (features[i].has_guid())
            out[i].guid() = features[i].guid();

    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Records
// ═══════════════════════════════════════════════════════════════════════════
Tombstone::Tombstone(
    const std::string& guid,
    const Item& obj,
    const std::string& collection,
    int obj_index,
    const std::optional<Xform>& xform,
    const std::optional<std::string>& parent_guid,
    int index,
    std::shared_ptr<TreeNode> node,
    const std::string& attribute,
    const std::vector<std::tuple<std::string, std::string, bool, std::string>>& edges
)
    : guid(guid), obj(obj), collection(collection), obj_index(obj_index), xform(xform), parent_guid(parent_guid),
      index(index), node(std::move(node)), attribute(attribute), edges(edges) {}

std::string Tombstone::str() const {
    return fmt::format("{}({})", kind, guid);
}

std::string Tombstone::repr() const {
    return fmt::format("{}({}, {}[{}])", kind, guid, collection, obj_index);
}

AddOp::AddOp(
    const std::string& guid,
    const Item& obj,
    const std::string& collection,
    int obj_index,
    const std::optional<Xform>& xform,
    const std::optional<std::string>& parent_guid,
    int index,
    std::shared_ptr<TreeNode> node,
    const std::string& attribute,
    const std::vector<std::tuple<std::string, std::string, bool, std::string>>& edges
)
    : Tombstone(guid, obj, collection, obj_index, xform, parent_guid, index, std::move(node), attribute, edges) {

    kind = "add";
}

RemoveOp::RemoveOp(
    const std::string& guid,
    const Item& obj,
    const std::string& collection,
    int obj_index,
    const std::optional<Xform>& xform,
    const std::optional<std::string>& parent_guid,
    int index,
    std::shared_ptr<TreeNode> node,
    const std::string& attribute,
    const std::vector<std::tuple<std::string, std::string, bool, std::string>>& edges
)
    : Tombstone(guid, obj, collection, obj_index, xform, parent_guid, index, std::move(node), attribute, edges) {

    kind = "remove";
}

ReplaceOp::ReplaceOp(const std::string& guid, const Item& before, const Item& after)
    : guid(guid), before(before), after(after) {}

std::string ReplaceOp::str() const {
    return fmt::format("replace({})", guid);
}

std::string ReplaceOp::repr() const {
    return fmt::format("replace({})", guid);
}

XformOp::XformOp(const std::string& guid, const std::optional<Xform>& before, const std::optional<Xform>& after)
    : guid(guid), before(before), after(after) {}

std::string XformOp::str() const {
    return fmt::format("xform({})", guid);
}

std::string XformOp::repr() const {
    return fmt::format("xform({})", guid);
}

DefinitionOp::DefinitionOp(
    const std::string& guid,
    const std::optional<Geometry>& before,
    const std::optional<Geometry>& after
)
    : guid(guid), before(before), after(after) {}

std::string DefinitionOp::str() const {
    return fmt::format("definition({})", guid);
}

std::string DefinitionOp::repr() const {
    return fmt::format("definition({})", guid);
}

Transaction::Transaction(std::string label) : label(std::move(label)) {}

std::string Transaction::str() const {
    return fmt::format("Transaction({}, {} ops)", label, ops.size());
}

std::string Transaction::repr() const {
    return fmt::format("Transaction({}, {} ops)", label, ops.size());
}

// ═══════════════════════════════════════════════════════════════════════════
// History
// ═══════════════════════════════════════════════════════════════════════════
bool History::can_undo() const {
    return !undo_stack.empty();
}

bool History::can_redo() const {
    return !redo_stack.empty();
}

int History::depth() const {
    return static_cast<int>(undo_stack.size());
}

void History::begin(const std::string& label) {

    commit();
    current = Transaction(label);
}

void History::commit() {

    std::optional<Transaction> transaction = std::move(current);
    current.reset();

    if (!transaction || transaction->ops.empty())
        return;

    undo_stack.push_back(std::move(*transaction));
    redo_stack.clear();

    if (depth() > CAPACITY)
        undo_stack.erase(undo_stack.begin());
}

void History::record(const Op& op) {

    if (!current)
        return;

    current->ops.push_back(op);
}

bool History::undo(Session& session) {

    commit();

    if (undo_stack.empty())
        return false;

    Transaction transaction = std::move(undo_stack.back());
    undo_stack.pop_back();

    for (int i = static_cast<int>(transaction.ops.size()) - 1; i >= 0; --i)
        _revert(transaction.ops[i], session);

    redo_stack.push_back(std::move(transaction));

    return true;
}

bool History::redo(Session& session) {

    commit();

    if (redo_stack.empty())
        return false;

    Transaction transaction = std::move(redo_stack.back());
    redo_stack.pop_back();

    for (size_t i = 0; i < transaction.ops.size(); ++i)
        _apply(transaction.ops[i], session);

    undo_stack.push_back(std::move(transaction));

    return true;
}

void History::clear() {

    undo_stack.clear();
    redo_stack.clear();
    current.reset();
}

void History::_revert(const Op& op, Session& session) {

    if (const AddOp* add = std::get_if<AddOp>(&op))
        session._detach(add->guid);
    else if (const RemoveOp* remove = std::get_if<RemoveOp>(&op))
        session._attach(*remove);
    else if (const ReplaceOp* replace = std::get_if<ReplaceOp>(&op))
        session._swap(replace->guid, clone(replace->before));
    else if (const XformOp* xform = std::get_if<XformOp>(&op))
        session._place(xform->guid, xform->before);
    else if (const DefinitionOp* definition = std::get_if<DefinitionOp>(&op))
        session._define(definition->guid, definition->before ? std::optional<Geometry>(clone(*definition->before)) : std::nullopt);
}

void History::_apply(const Op& op, Session& session) {

    if (const AddOp* add = std::get_if<AddOp>(&op))
        session._attach(*add);
    else if (const RemoveOp* remove = std::get_if<RemoveOp>(&op))
        session._detach(remove->guid);
    else if (const ReplaceOp* replace = std::get_if<ReplaceOp>(&op))
        session._swap(replace->guid, clone(replace->after));
    else if (const XformOp* xform = std::get_if<XformOp>(&op))
        session._place(xform->guid, xform->after);
    else if (const DefinitionOp* definition = std::get_if<DefinitionOp>(&op))
        session._define(definition->guid, definition->after ? std::optional<Geometry>(clone(*definition->after)) : std::nullopt);
}

std::string History::str() const {
    return fmt::format("History({} undo, {} redo)", undo_stack.size(), redo_stack.size());
}

std::string History::repr() const {
    return fmt::format("History({} undo, {} redo)", undo_stack.size(), redo_stack.size());
}

} // namespace session_cpp
