# Undo, history and purge

How `Session` deletes, undoes, redoes and saves. The code is in `src/session.h`, `src/history.h`, `src/collection.h`, `src/tree.h` and `src/graph.h`.

## A delete is a flag, not a copy

`remove_object(guid)` does not erase anything. It marks the object dead in place: its slot in the typed list, its tree node, its transform, its graph vertex and edges, and the interactions on those edges.

A dead entry is still in memory but hidden from every public view.

A tomb is the small record that remembers where a dead object lives (its list, its slot, its node) and holds what it parked while dead (transform, vertex, edges, interactions).

History keeps the same `shared_ptr` the session held, never a copy. Undo flips the tomb back to live, so the object returns as the same pointer, in the same slot, under the same tree node. Redo flips it dead again. Both are O(1) plus the object's edges.

Tree nodes work the same way: a dead node hides itself and its subtree, and a node moved to a new parent leaves a dead ghost in its old place.

## Collection

Every typed list in `Objects` (`points`, `meshes`, `breps`, ...) is a `Collection<E>`: a vector of slots where each slot can be live or dead.

The public views see live entries only, in slot order:

- `size()` and `empty()` count live entries.
- `begin()` / `end()` and range-for skip dead slots.
- `at(i)` and `[i]` return the i-th live entry and throw `std::out_of_range` past the end.
- `front()`, `back()` and `to_vector()` also see live entries only.

The raw slot API is for `Session`, `History` and the purge, not for user code: `get_slot`, `get_item`, `set_item`, `is_dead`, `set_dead`, `get_tomb`, `set_tomb`, `number_of_dead`, `number_of_slots`, `is_compacting`, `compact_step` and `compact`.

Copying a collection copies its live entries only, already compacted.

## History is bounded by bytes

Every record carries a weight: an estimate of the bytes it keeps alive. A record costs `RECORD` (256 bytes) plus the weight of the object it holds; a removed object also adds 128 bytes per graph edge.

For a mesh with V vertices and F faces the weight is `128 + 768·V + 192·F`. A mesh of 10,000 vertices and 20,000 faces weighs about 11.5 MB.

When the undo and redo stacks together pass `history.budget` (`BUDGET`, 256 MB by default), the oldest step is dropped. Steps are also capped at `CAPACITY` (64), but for real models the byte budget is the limit that bites. The newest step is always kept.

A new step clears the redo stack.

## Purge

A dead slot stays until a purge frees it. A purge only frees what no history record can still reach.

- `purge()` clears the history, then compacts everything: every list, every tree node, and dense graph indices. It is O(n + N + V log V + E log E).
- `pb_dumps`, `pb_dump`, `file_json_dumps` and `file_json_dump` call `purge()` first, so undo cannot cross a save. `to_proto` and `jsondump` are `const`: they write live entries and keep the history.
- `purge_step(work)` does at most `work` units of the purge and returns `true` while it is unfinished. Call it on idle frames with `PURGE_WORK` (16384 units, about 2 ms). It only runs when dropped history left something to free (`purge_due()`).
- `checkpoint(work)` writes the live session as protobuf bytes, `work` units per call, and returns them once done. It keeps the history, so it is the call for autosave.

Both are resumable cursors and never block an edit. A compaction picks up where it stopped. A checkpoint started before an edit is thrown away and restarts from the new state.

## Twins

A twin is a second entry with a guid that is already live.

`add_*` refuses a guid that is already live anywhere in the session, as an object, component, instance or definition. It adds nothing and returns `nullptr` or the node that guid already has.

Ownership is judged session-wide: when undo or redo would revive a tomb whose guid another live entry now owns, the tomb stays dead and the live entry keeps the guid, its node, transform and edges.

## Interactions

`interactions` maps an edge guid to that edge's list of interactions. A delete parks each incident edge's list in the tomb under the edge guid. Undo puts the edges back and returns each list only when the restored edge has the same guid. Redo parks them again.

## Graph: take_node and put_node

- `take_node(key)` removes a vertex and its incident edges without renumbering the others, and returns them; `std::nullopt` when the key is missing.
- `put_node(vertex, edges)` puts them back. It adopts a bare vertex `add_edge` created meanwhile, and skips an edge whose other end is gone or whose place a newer edge took.

Both are O(d log V) for a vertex of degree d. Indices become dense again at the next `purge()`, which calls `renumber()`.

## Example

```cpp
#include "session.h"
using namespace session_cpp;

Session session;
std::shared_ptr<TreeNode> node = session.add_point(std::make_shared<Point>(1.0, 2.0, 3.0));
const std::string guid = node->name;

session.begin("delete");
session.remove_object(guid);
session.commit();
// session.objects.points->size() == 0, number_of_slots() == 1: the slot is dead, not gone.

session.undo(); // The same pointer is live again in the same slot.
session.redo(); // Dead again.

session.pb_dump("scene.pb"); // Purges: history cleared, the dead slot freed.
// session.undo() now returns false.

// On an idle frame:
session.purge_step(PURGE_WORK);

// Autosave, one call per frame until the bytes arrive:
std::optional<std::string> bytes = session.checkpoint(PURGE_WORK);
```
