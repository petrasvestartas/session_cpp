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

/// Bytes a mesh pins, from its counts: a vertex owns its halfedge map, a face its vertex list (measured, triangle caches not counted).
static size_t mesh_weight(const Mesh& mesh) {
    return 128 + 768 * mesh.number_of_vertices() + 192 * mesh.number_of_faces();
}

/// Bytes a curve pins, from its control point and knot counts.
static size_t curve_weight(const NurbsCurve& curve) {
    return 256 + 32 * curve.cv_count() + 8 * curve.m_nurbsknot.size();
}

/// Bytes a surface pins, from its control point and knot counts.
static size_t surface_weight(const NurbsSurface& surface) {
    return 1536 + 32 * surface.cv_count() + 8 * (surface.m_nurbsknot[0].size() + surface.m_nurbsknot[1].size());
}

/// Bytes a brep pins: every surface and curve of its pools, plus its tables.
static size_t brep_weight(const BRep& brep) {

    size_t bytes = 512 + 24 * brep.m_vertices.size() + 64 * (brep.m_edges.size() + brep.m_faces.size());

    for (const NurbsSurface& surface : brep.m_surfaces)
        bytes += surface_weight(surface);

    for (const NurbsCurve& curve : brep.m_curves_3d)
        bytes += curve_weight(curve);

    for (const NurbsCurve& curve : brep.m_curves_2d)
        bytes += curve_weight(curve);

    return bytes;
}

/// Bytes an element pins: its record, its mesh or brep and its features.
static size_t element_weight(const Element& element) {

    size_t geometry = 0;

    if (element.geometry_type_name() == "Mesh")
        geometry = mesh_weight(element.geometry_mesh());
    else if (element.geometry_type_name() == "BRep")
        geometry = brep_weight(element.geometry_brep());

    return 256 + geometry + 128 * element.features_count();
}

size_t weight(const Item& item) {

    if (std::holds_alternative<Component>(item))
        return 128;

    if (const std::shared_ptr<InstanceRef>* instance = std::get_if<std::shared_ptr<InstanceRef>>(&item))
        return 256 + 128 * (*instance)->features.size();

    const Geometry& geometry = std::get<Geometry>(item);

    if (std::holds_alternative<std::shared_ptr<Point>>(geometry))
        return 64;

    if (std::holds_alternative<std::shared_ptr<Line>>(geometry))
        return 96;

    if (std::holds_alternative<std::shared_ptr<Plane>>(geometry))
        return 160;

    if (std::holds_alternative<std::shared_ptr<OBB>>(geometry))
        return 192;

    if (const std::shared_ptr<Polyline>* polyline = std::get_if<std::shared_ptr<Polyline>>(&geometry))
        return 64 + 24 * (*polyline)->point_count();

    if (const std::shared_ptr<PointCloud>* cloud = std::get_if<std::shared_ptr<PointCloud>>(&geometry))
        return 64 + 24 * (*cloud)->point_count() + 24 * (*cloud)->normal_count() + 16 * (*cloud)->color_count();

    if (const std::shared_ptr<Mesh>* mesh = std::get_if<std::shared_ptr<Mesh>>(&geometry))
        return mesh_weight(**mesh);

    if (const std::shared_ptr<NurbsCurve>* curve = std::get_if<std::shared_ptr<NurbsCurve>>(&geometry))
        return curve_weight(**curve);

    if (const std::shared_ptr<NurbsSurface>* surface = std::get_if<std::shared_ptr<NurbsSurface>>(&geometry))
        return surface_weight(**surface);

    if (const std::shared_ptr<BRep>* brep = std::get_if<std::shared_ptr<BRep>>(&geometry))
        return brep_weight(**brep);

    return element_weight(*std::get<std::shared_ptr<Element>>(geometry));
}

// ═══════════════════════════════════════════════════════════════════════════
// Records
// ═══════════════════════════════════════════════════════════════════════════
Tomb::Tomb(const std::string& collection, bool definition, size_t slot, std::shared_ptr<TreeNode> node)
    : collection(collection), definition(definition), slot(slot), node(std::move(node)) {}

Tombstone::Tombstone(
    const std::string& guid,
    const std::string& collection,
    const std::optional<std::string>& parent_guid,
    int index,
    std::shared_ptr<TreeNode> node,
    std::shared_ptr<Tomb> tomb
)
    : guid(guid), collection(collection), parent_guid(parent_guid), index(index), node(std::move(node)),
      tomb(std::move(tomb)) {}

std::string Tombstone::str() const {
    return fmt::format("{}({})", kind, guid);
}

std::string Tombstone::repr() const {
    return fmt::format("{}({}, {})", kind, guid, collection);
}

AddOp::AddOp(
    const std::string& guid,
    const std::string& collection,
    const std::optional<std::string>& parent_guid,
    int index,
    std::shared_ptr<TreeNode> node,
    std::shared_ptr<Tomb> tomb
)
    : Tombstone(guid, collection, parent_guid, index, std::move(node), std::move(tomb)) {

    kind = "add";
}

RemoveOp::RemoveOp(
    const std::string& guid,
    const std::string& collection,
    const std::optional<std::string>& parent_guid,
    int index,
    std::shared_ptr<TreeNode> node,
    std::shared_ptr<Tomb> tomb
)
    : Tombstone(guid, collection, parent_guid, index, std::move(node), std::move(tomb)) {

    kind = "remove";
}

Entry::Entry(bool definition, std::shared_ptr<TreeNode> node, std::shared_ptr<Tomb> tomb)
    : definition(definition), node(std::move(node)), tomb(std::move(tomb)) {}

ReplaceOp::ReplaceOp(const std::string& guid, const Item& before, const Item& after, Entry entry)
    : guid(guid), before(before), after(after), entry(std::move(entry)) {}

std::string ReplaceOp::str() const {
    return fmt::format("replace({})", guid);
}

std::string ReplaceOp::repr() const {
    return fmt::format("replace({})", guid);
}

XformOp::XformOp(
    const std::string& guid,
    const std::optional<Xform>& before,
    const std::optional<Xform>& after,
    std::shared_ptr<TreeNode> node
)
    : guid(guid), before(before), after(after), node(std::move(node)) {}

std::string XformOp::str() const {
    return fmt::format("xform({})", guid);
}

std::string XformOp::repr() const {
    return fmt::format("xform({})", guid);
}

TreeOp::TreeOp(
    const std::string& guid,
    std::shared_ptr<TreeNode> node,
    std::shared_ptr<Tomb> tomb,
    std::shared_ptr<TreeNode> ghost,
    const std::string& name_before,
    const std::string& name_after,
    std::optional<Color> color_before,
    std::optional<Color> color_after,
    bool dead_before,
    bool dead_after
)
    : guid(guid), node(std::move(node)), tomb(std::move(tomb)), ghost(std::move(ghost)), name_before(name_before),
      name_after(name_after), color_before(std::move(color_before)), color_after(std::move(color_after)),
      dead_before(dead_before), dead_after(dead_after) {}

std::string TreeOp::str() const {
    return fmt::format("tree({})", guid);
}

std::string TreeOp::repr() const {
    return fmt::format("tree({})", guid);
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

    for (const Transaction& undone : redo_stack)
        dropped += undone.ops.size();

    redo_stack.clear();
    bytes = _pinned();

    while (undo_stack.size() > 1 && (depth() > CAPACITY || bytes > budget)) {
        dropped += undo_stack.front().ops.size();
        bytes -= undo_stack.front().bytes;
        undo_stack.erase(undo_stack.begin());
    }
}

void History::record(Op op, size_t bytes) {

    if (!current)
        return;

    current->ops.push_back(std::move(op));
    current->bytes += bytes;
    this->bytes += bytes;
}

bool History::abort(Session& session) {

    std::optional<Transaction> transaction = std::move(current);
    current.reset();

    if (!transaction)
        return false;

    for (int i = static_cast<int>(transaction->ops.size()) - 1; i >= 0; --i)
        _revert(transaction->ops[i], session);

    dropped += transaction->ops.size();
    bytes = _pinned();

    return true;
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

    for (const Transaction& transaction : undo_stack)
        dropped += transaction.ops.size();

    for (const Transaction& transaction : redo_stack)
        dropped += transaction.ops.size();

    if (current)
        dropped += current->ops.size();

    undo_stack.clear();
    redo_stack.clear();
    current.reset();
    bytes = 0;
}

size_t History::_pinned() const {

    size_t pinned = 0;

    for (const Transaction& transaction : undo_stack)
        pinned += transaction.bytes;

    for (const Transaction& transaction : redo_stack)
        pinned += transaction.bytes;

    return pinned;
}

void History::_revert(const Op& op, Session& session) {

    if (const AddOp* add = std::get_if<AddOp>(&op))
        session._kill(add->tomb);
    else if (const RemoveOp* remove = std::get_if<RemoveOp>(&op))
        session._revive(remove->tomb);
    else if (const ReplaceOp* replace = std::get_if<ReplaceOp>(&op))
        session._swap(replace->guid, replace->before, replace->entry);
    else if (const XformOp* xform = std::get_if<XformOp>(&op))
        session._place(xform->guid, xform->before, xform->node);
    else if (const TreeOp* tree = std::get_if<TreeOp>(&op))
        session._tree(*tree, true);
}

void History::_apply(const Op& op, Session& session) {

    if (const AddOp* add = std::get_if<AddOp>(&op))
        session._revive(add->tomb);
    else if (const RemoveOp* remove = std::get_if<RemoveOp>(&op))
        session._kill(remove->tomb);
    else if (const ReplaceOp* replace = std::get_if<ReplaceOp>(&op))
        session._swap(replace->guid, replace->after, replace->entry);
    else if (const XformOp* xform = std::get_if<XformOp>(&op))
        session._place(xform->guid, xform->after, xform->node);
    else if (const TreeOp* tree = std::get_if<TreeOp>(&op))
        session._tree(*tree, false);
}

std::string History::str() const {
    return fmt::format("History({} undo, {} redo)", undo_stack.size(), redo_stack.size());
}

std::string History::repr() const {
    return fmt::format("History({} undo, {} redo)", undo_stack.size(), redo_stack.size());
}

} // namespace session_cpp
