#include "history.h"
#include "session.h"
#include "fmt/core.h"

namespace session_cpp {

Item clone(const Item& obj) {
  return std::visit([](const auto& stored) -> Item {
    using T = std::decay_t<decltype(stored)>;
    if constexpr (std::is_same_v<T, Component>) {
      Component snapshot = stored;
      snapshot.guid() = stored.guid();
      return snapshot;
    } else {
      return std::visit([](const auto& live) -> Geometry {
        using P = typename std::decay_t<decltype(live)>::element_type;
        std::string guid = live->guid();
        auto snapshot = std::make_shared<P>(*live);
        snapshot->guid() = guid;
        return snapshot;
      }, stored);
    }
  }, obj);
}

// ═══════════════════════════════════════════════════════════════════════════
// Records
// ═══════════════════════════════════════════════════════════════════════════

Tombstone::Tombstone(const std::string& guid, const Item& obj, const std::string& collection, int obj_index, const std::optional<Xform>& xform, const std::optional<std::string>& parent_guid, int index, std::shared_ptr<TreeNode> node, const std::string& attribute, const std::vector<std::tuple<std::string, std::string, bool>>& edges)
    : guid(guid), obj(obj), collection(collection), obj_index(obj_index), xform(xform), parent_guid(parent_guid), index(index), node(std::move(node)), attribute(attribute), edges(edges) {}

std::string Tombstone::str() const { return fmt::format("{}({})", kind, guid); }

std::string Tombstone::repr() const { return fmt::format("{}({}, {}[{}])", kind, guid, collection, obj_index); }

AddOp::AddOp(const std::string& guid, const Item& obj, const std::string& collection, int obj_index, const std::optional<Xform>& xform, const std::optional<std::string>& parent_guid, int index, std::shared_ptr<TreeNode> node, const std::string& attribute, const std::vector<std::tuple<std::string, std::string, bool>>& edges)
    : Tombstone(guid, obj, collection, obj_index, xform, parent_guid, index, std::move(node), attribute, edges) {
  kind = "add";
}

RemoveOp::RemoveOp(const std::string& guid, const Item& obj, const std::string& collection, int obj_index, const std::optional<Xform>& xform, const std::optional<std::string>& parent_guid, int index, std::shared_ptr<TreeNode> node, const std::string& attribute, const std::vector<std::tuple<std::string, std::string, bool>>& edges)
    : Tombstone(guid, obj, collection, obj_index, xform, parent_guid, index, std::move(node), attribute, edges) {
  kind = "remove";
}

ReplaceOp::ReplaceOp(const std::string& guid, const Item& before, const Item& after) : guid(guid), before(before), after(after) {}

std::string ReplaceOp::str() const { return fmt::format("replace({})", guid); }

std::string ReplaceOp::repr() const { return fmt::format("replace({})", guid); }

XformOp::XformOp(const std::string& guid, const std::optional<Xform>& before, const std::optional<Xform>& after) : guid(guid), before(before), after(after) {}

std::string XformOp::str() const { return fmt::format("xform({})", guid); }

std::string XformOp::repr() const { return fmt::format("xform({})", guid); }

Transaction::Transaction(std::string label) : label(std::move(label)) {}

std::string Transaction::str() const { return fmt::format("Transaction({}, {} ops)", label, ops.size()); }

std::string Transaction::repr() const { return fmt::format("Transaction({}, {} ops)", label, ops.size()); }

// ═══════════════════════════════════════════════════════════════════════════
// History
// ═══════════════════════════════════════════════════════════════════════════

bool History::can_undo() const { return !undo_stack.empty(); }

bool History::can_redo() const { return !redo_stack.empty(); }

int History::depth() const { return static_cast<int>(undo_stack.size()); }

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
  std::visit([&](const auto& record) {
    using T = std::decay_t<decltype(record)>;
    if constexpr (std::is_same_v<T, AddOp>)
      session._detach(record.guid);
    else if constexpr (std::is_same_v<T, RemoveOp>)
      session._attach(record);
    else if constexpr (std::is_same_v<T, ReplaceOp>)
      session._swap(record.guid, clone(record.before));
    else
      session._place(record.guid, record.before);
  }, op);
}

void History::_apply(const Op& op, Session& session) {
  std::visit([&](const auto& record) {
    using T = std::decay_t<decltype(record)>;
    if constexpr (std::is_same_v<T, AddOp>)
      session._attach(record);
    else if constexpr (std::is_same_v<T, RemoveOp>)
      session._detach(record.guid);
    else if constexpr (std::is_same_v<T, ReplaceOp>)
      session._swap(record.guid, clone(record.after));
    else
      session._place(record.guid, record.after);
  }, op);
}

std::string History::str() const { return fmt::format("History({} undo, {} redo)", undo_stack.size(), redo_stack.size()); }

std::string History::repr() const { return fmt::format("History({} undo, {} redo)", undo_stack.size(), redo_stack.size()); }

} // namespace session_cpp
