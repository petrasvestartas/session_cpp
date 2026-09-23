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

inline constexpr int CAPACITY = 64; // Committed transactions kept; past it the oldest is dropped.

/// A deep copy that keeps the guid, which `duplicate()` and most copy constructors would mint anew.
Item clone(const Item& obj);

/// A deep copy of geometry that keeps its guid and type, element feature guids included.
Geometry clone(const Geometry& obj);

/// A copy of features that keeps each guid, which the ElementFeature copy would mint anew.
std::vector<ElementFeature> clone(const std::vector<ElementFeature>& features);

// ═══════════════════════════════════════════════════════════════════════════
// Records
// ═══════════════════════════════════════════════════════════════════════════

/// Everything needed to put one object back into every live table of a session.
class Tombstone {
public:
    std::string kind = "";                  // "add" or "remove".
    std::string guid;                       // The object's guid; the clone carries the same one.
    Item obj;                               // A clone() of the object, never the live instance.
    std::string collection;                 // The Objects list it lives in: "points", "lines", ... "components".
    int obj_index;                          // Its position in that list, so the order() sequence survives a round trip.
    std::optional<Xform> xform;             // Its local transform, nullopt when none was set.
    std::optional<std::string> parent_guid; // Name of its tree parent, nullopt when it was added without one.
    int index;                              // Its position among the parent's children.
    std::shared_ptr<TreeNode> node;         // The detached tree node with its whole subtree, nullptr for an add.
    std::string attribute;                  // Its graph node attribute.
    std::vector<std::tuple<std::string, std::string, bool, std::string>> edges; // Incident edges as (other guid, attribute, forward, edge guid or "").

    /// Construct from every field of the kit.
    Tombstone(
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
    );

    /// Return a string representation of the record.
    std::string str() const;

    /// Return a string representation of the record for debugging.
    std::string repr() const;
};

/// An object entered the session; undo detaches it, redo attaches the kit again.
class AddOp : public Tombstone {
public:
    /// Construct an add record from every field of the kit.
    AddOp(
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
    );
};

/// An object left the session; the kit is what brings it back on undo.
class RemoveOp : public Tombstone {
public:
    /// Construct a remove record from every field of the kit.
    RemoveOp(
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
    );
};

/// The object under `guid` was swapped: absolute before/after snapshots, never deltas.
class ReplaceOp {
public:
    std::string kind = "replace"; // Always "replace".
    std::string guid;             // The object's guid.
    Item before;                  // Snapshot before the swap.
    Item after;                   // Snapshot after the swap.

    /// Construct from the guid and the before and after snapshots.
    ReplaceOp(const std::string& guid, const Item& before, const Item& after);

    /// Return a string representation of the record.
    std::string str() const;

    /// Return a string representation of the record for debugging.
    std::string repr() const;
};

/// The local transform under `guid` changed; nullopt on either side means "none set".
class XformOp {
public:
    std::string kind = "xform";  // Always "xform".
    std::string guid;            // The object's guid.
    std::optional<Xform> before; // Transform before the change.
    std::optional<Xform> after;  // Transform after the change.

    /// Construct from the guid and the before and after transforms.
    XformOp(const std::string& guid, const std::optional<Xform>& before, const std::optional<Xform>& after);

    /// Return a string representation of the record.
    std::string str() const;

    /// Return a string representation of the record for debugging.
    std::string repr() const;
};

/// A definition added (nullopt before), removed (nullopt after) or replaced.
class DefinitionOp {
public:
    std::string kind = "definition";      // Always "definition".
    std::string guid;                     // The definition's guid.
    std::optional<Geometry> before;       // Snapshot before, nullopt when it was added.
    std::optional<Geometry> after;        // Snapshot after, nullopt when it was removed.

    /// Construct from the guid and the before and after snapshots.
    DefinitionOp(const std::string& guid, const std::optional<Geometry>& before, const std::optional<Geometry>& after);

    /// Return a string representation of the record.
    std::string str() const;

    /// Return a string representation of the record for debugging.
    std::string repr() const;
};

/// Any one recorded op.
using Op = std::variant<AddOp, RemoveOp, ReplaceOp, XformOp, DefinitionOp>;

/// One undoable step: a label and the ops it made, in the order they happened.
class Transaction {
public:
    std::string label;   // What the step did.
    std::vector<Op> ops; // Ops in the order they happened.

    /// Construct an empty transaction with a label.
    Transaction(std::string label = "my_transaction");

    /// Return a string representation of the transaction.
    std::string str() const;

    /// Return a string representation of the transaction for debugging.
    std::string repr() const;
};

// ═══════════════════════════════════════════════════════════════════════════
// History
// ═══════════════════════════════════════════════════════════════════════════

/// CAD-style undo/redo over a Session, in memory only: records exist between `begin` and `commit`, every save purges them.
class History {
public:
    std::vector<Transaction> undo_stack; // Committed transactions, oldest first; capped at CAPACITY.
    std::vector<Transaction> redo_stack; // Undone transactions, cleared the moment a new transaction commits.
    std::optional<Transaction> current;  // The open transaction, nullopt between commit and the next begin.

    /// Return whether a committed transaction can be undone.
    bool can_undo() const;

    /// Return whether an undone transaction can be redone.
    bool can_redo() const;

    /// Return the number of committed transactions.
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

    /// Drop every transaction, open or committed.
    void clear();

    /// Return a string representation of the history.
    std::string str() const;

    /// Return a string representation of the history for debugging.
    std::string repr() const;

private:
    /// Undo one op against the session.
    void _revert(const Op& op, Session& session);

    /// Redo one op against the session.
    void _apply(const Op& op, Session& session);
};

} // namespace session_cpp
