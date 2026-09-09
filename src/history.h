#pragma once
#include "objects.h"
#include "tree.h"
#include "xform.h"
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

namespace session_cpp {

class Session;

inline constexpr int CAPACITY = 64; ///< Committed transactions kept; past it the oldest is dropped

/// A deep copy that KEEPS the guid: a snapshot must still name the object it stands for.
/// `duplicate()` mints a fresh guid on purpose, and so does the copy constructor of most
/// geometry types, which is why the guid is put back after the copy.
Item clone(const Item& obj);

// ═══════════════════════════════════════════════════════════════════════════
// Records
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @class Tombstone
 * @brief Everything needed to put ONE object back into every live table of a session.
 */
class Tombstone {
public:
  std::string kind = "";                  ///< "add" or "remove"
  std::string guid;                       ///< The object's guid; the clone carries the same one
  Item obj;                               ///< A clone() of the object, never the live instance
  std::string collection;                 ///< The Objects list it lives in: "points", "lines", ... "components"
  int obj_index;                          ///< Its position in that list, so the order() sequence survives a round trip
  std::optional<Xform> xform;             ///< Its local transform, nullopt when none was set
  std::optional<std::string> parent_guid; ///< Name of its tree parent, nullopt when it was added without one
  int index;                              ///< Its position among the parent's children
  std::shared_ptr<TreeNode> node;         ///< The detached tree node with its whole subtree, nullptr for an add
  std::string attribute;                  ///< Its graph node attribute
  std::vector<std::tuple<std::string, std::string, bool>> edges; ///< Incident edges as (other_guid, attribute, forward), forward when the object was the edge's v0

  Tombstone(const std::string& guid, const Item& obj, const std::string& collection, int obj_index, const std::optional<Xform>& xform, const std::optional<std::string>& parent_guid, int index, std::shared_ptr<TreeNode> node, const std::string& attribute, const std::vector<std::tuple<std::string, std::string, bool>>& edges);

  std::string str() const;
  std::string repr() const;
};

/// An object entered the session; undo detaches it, redo attaches the kit again.
class AddOp : public Tombstone {
public:
  AddOp(const std::string& guid, const Item& obj, const std::string& collection, int obj_index, const std::optional<Xform>& xform, const std::optional<std::string>& parent_guid, int index, std::shared_ptr<TreeNode> node, const std::string& attribute, const std::vector<std::tuple<std::string, std::string, bool>>& edges);
};

/// An object left the session; the kit is what brings it back on undo.
class RemoveOp : public Tombstone {
public:
  RemoveOp(const std::string& guid, const Item& obj, const std::string& collection, int obj_index, const std::optional<Xform>& xform, const std::optional<std::string>& parent_guid, int index, std::shared_ptr<TreeNode> node, const std::string& attribute, const std::vector<std::tuple<std::string, std::string, bool>>& edges);
};

/// The object under `guid` was swapped: absolute before/after snapshots, never deltas.
class ReplaceOp {
public:
  std::string kind = "replace";
  std::string guid;
  Item before;
  Item after;

  ReplaceOp(const std::string& guid, const Item& before, const Item& after);

  std::string str() const;
  std::string repr() const;
};

/// The local transform under `guid` changed; nullopt on either side means "none set".
class XformOp {
public:
  std::string kind = "xform";
  std::string guid;
  std::optional<Xform> before;
  std::optional<Xform> after;

  XformOp(const std::string& guid, const std::optional<Xform>& before, const std::optional<Xform>& after);

  std::string str() const;
  std::string repr() const;
};

using Op = std::variant<AddOp, RemoveOp, ReplaceOp, XformOp>;

/// One undoable step: a label and the ops it made, in the order they happened.
class Transaction {
public:
  std::string label;
  std::vector<Op> ops;

  Transaction(std::string label = "my_transaction");

  std::string str() const;
  std::string repr() const;
};

// ═══════════════════════════════════════════════════════════════════════════
// History
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @class History
 * @brief CAD-style undo/redo over a Session, in memory only.
 *
 * A removed object leaves every live table at once; its RemoveOp is the tombstone that
 * carries the resurrection kit. Records are only written while a transaction is open
 * (`begin` ... `commit`), and every save purges the buffer, as Rhino does: history never
 * crosses pb or JSON, and a loaded session starts with an empty one.
 */
class History {
public:
  std::vector<Transaction> undo_stack;  ///< Committed transactions, oldest first; capped at CAPACITY
  std::vector<Transaction> redo_stack;  ///< Undone transactions, cleared the moment a new transaction commits
  std::optional<Transaction> current;   ///< The open transaction, nullopt between commit and the next begin

  bool can_undo() const;
  bool can_redo() const;
  int depth() const;

  /// Open a transaction; an already open one is committed first so no op is lost.
  void begin(const std::string& label);

  /// Close the open transaction. An empty one is dropped; a real one clears redo.
  void commit();

  /// Append an op to the open transaction; a no-op when none is open.
  void record(const Op& op);

  /// Revert the newest transaction, ops in reverse order, and park it for redo.
  bool undo(Session& session);

  /// Re-apply the newest undone transaction, ops in their original order.
  bool redo(Session& session);

  void clear();

  std::string str() const;
  std::string repr() const;

private:
  void _revert(const Op& op, Session& session);
  void _apply(const Op& op, Session& session);
};

} // namespace session_cpp
