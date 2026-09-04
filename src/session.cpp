#include "session.h"
#include "graph.h"
#include "tree.h"
#include "intersection.h"
#include "session.pb.h"
#include <algorithm>
#include <functional>
#include <set>

/**
 * ADDING NEW GEOMETRY TYPES - CHECKLIST
 * =====================================
 * 
 * When adding a new geometry type (e.g., Sphere, Cone, etc.), update:
 * 
 * 1. SESSION.H:
 *    - Add to Geometry variant (e.g., std::shared_ptr<Sphere>)
 *    - Add collection to Objects class (e.g., std::shared_ptr<std::vector<std::shared_ptr<Sphere>>> spheres)
 *    - Add to Objects constructor initialization
 *    - Declare Session::add_XXX() method
 * 
 * 2. SESSION.CPP:
 *    - Implement Session::add_XXX() method
 *    - Add case in Session::compute_bounding_box() visitor
 *    - Add case in Session::ray_intersect_geometry() visitor
 *    - Add collection to Session::get_geometry() transformation loop
 *    - Add collection to Session::jsonload() rebuild lookup
 * 
 * 3. OBJECTS.H/CPP:
 *    - Add collection member and initialization
 *    - Add to Objects::jsondump()
 *    - Add to Objects::jsonload()
 *    - Add to Objects::to_string()
 * 
 * 4. INTERSECTION.H/CPP (if needed):
 *    - Implement ray-geometry intersection method
 *    - Consider adding to existing methods if applicable
 * 
 * Critical visitor patterns to update:
 * - compute_bounding_box(): Define AABB for collision detection
 * - ray_intersect_geometry(): Define precise ray intersection logic
 * - get_geometry(): Apply transformations to new type
 */

namespace session_cpp {

std::string Session::str() const {
  return fmt::format("Session(name={}, objects={}, tree={}, graph={})", name,
                     objects.str(), tree.str(), graph.str());
}

// Geometry Management

std::shared_ptr<TreeNode> Session::add_point(std::shared_ptr<Point> point, std::shared_ptr<TreeNode> parent) {
  objects.points->push_back(point);
  lookup[point->guid()] = point;
  graph.add_node(point->guid(), "point_" + point->name);
  cache_geometry_aabb(point->guid(), point);
  auto node = std::make_shared<TreeNode>(point->guid());
  if (parent) add(node, parent);
  return node;
}

std::shared_ptr<TreeNode> Session::add_line(std::shared_ptr<Line> line, std::shared_ptr<TreeNode> parent) {
  objects.lines->push_back(line);
  lookup[line->guid()] = line;
  graph.add_node(line->guid(), "line_" + line->name);
  cache_geometry_aabb(line->guid(), line);
  auto node = std::make_shared<TreeNode>(line->guid());
  if (parent) add(node, parent);
  return node;
}

std::shared_ptr<TreeNode> Session::add_plane(std::shared_ptr<Plane> plane, std::shared_ptr<TreeNode> parent) {
  objects.planes->push_back(plane);
  lookup[plane->guid()] = plane;
  graph.add_node(plane->guid(), "plane_" + plane->name);
  cache_geometry_aabb(plane->guid(), plane);
  auto node = std::make_shared<TreeNode>(plane->guid());
  if (parent) add(node, parent);
  return node;
}

std::shared_ptr<TreeNode> Session::add_obb(std::shared_ptr<OBB> bbox) {
  objects.bboxes->push_back(bbox);
  lookup[bbox->guid()] = bbox;
  graph.add_node(bbox->guid(), "bbox_" + bbox->name);
  cache_geometry_aabb(bbox->guid(), bbox);
  auto node = std::make_shared<TreeNode>(bbox->guid());
  return node;
}

std::shared_ptr<TreeNode> Session::add_polyline(std::shared_ptr<Polyline> polyline, std::shared_ptr<TreeNode> parent) {
  objects.polylines->push_back(polyline);
  lookup[polyline->guid()] = polyline;
  graph.add_node(polyline->guid(), "polyline_" + polyline->name);
  cache_geometry_aabb(polyline->guid(), polyline);
  auto node = std::make_shared<TreeNode>(polyline->guid());
  if (parent) add(node, parent);
  return node;
}

std::shared_ptr<TreeNode> Session::add_pointcloud(std::shared_ptr<PointCloud> pointcloud, std::shared_ptr<TreeNode> parent) {
  objects.pointclouds->push_back(pointcloud);
  lookup[pointcloud->guid()] = pointcloud;
  graph.add_node(pointcloud->guid(), "pointcloud_" + pointcloud->name);
  cache_geometry_aabb(pointcloud->guid(), pointcloud);
  auto node = std::make_shared<TreeNode>(pointcloud->guid());
  if (parent) add(node, parent);
  return node;
}

std::shared_ptr<TreeNode> Session::add_mesh(std::shared_ptr<Mesh> mesh, std::shared_ptr<TreeNode> parent) {
  objects.meshes->push_back(mesh);
  lookup[mesh->guid()] = mesh;
  graph.add_node(mesh->guid(), "mesh_" + mesh->name);
  cache_geometry_aabb(mesh->guid(), mesh);
  auto node = std::make_shared<TreeNode>(mesh->guid());
  if (parent) add(node, parent);
  return node;
}

std::shared_ptr<TreeNode> Session::add_nurbscurve(std::shared_ptr<NurbsCurve> nurbscurve, std::shared_ptr<TreeNode> parent) {
  objects.nurbscurves->push_back(nurbscurve);
  lookup[nurbscurve->guid()] = nurbscurve;
  graph.add_node(nurbscurve->guid(), "nurbscurve_" + nurbscurve->name);
  cache_geometry_aabb(nurbscurve->guid(), nurbscurve);
  auto node = std::make_shared<TreeNode>(nurbscurve->guid());
  if (parent) add(node, parent);
  return node;
}

std::shared_ptr<TreeNode> Session::add_nurbssurface(std::shared_ptr<NurbsSurface> nurbssurface, std::shared_ptr<TreeNode> parent) {
  objects.nurbssurfaces->push_back(nurbssurface);
  lookup[nurbssurface->guid()] = nurbssurface;
  graph.add_node(nurbssurface->guid(), "nurbssurface_" + nurbssurface->name);
  cache_geometry_aabb(nurbssurface->guid(), nurbssurface);
  auto node = std::make_shared<TreeNode>(nurbssurface->guid());
  if (parent) add(node, parent);
  return node;
}

std::shared_ptr<TreeNode> Session::add_brep(std::shared_ptr<BRep> brep, std::shared_ptr<TreeNode> parent) {
  objects.breps->push_back(brep);
  lookup[brep->guid()] = brep;
  bvh_cache_dirty = true;
  graph.add_node(brep->guid(), "brep_" + brep->name);
  auto node = std::make_shared<TreeNode>(brep->guid());
  if (parent) add(node, parent);
  return node;
}

std::shared_ptr<TreeNode> Session::add_element(std::shared_ptr<Element> element, std::shared_ptr<TreeNode> parent) {
  objects.elements->push_back(element);
  lookup[element->guid()] = element;
  bvh_cache_dirty = true;
  graph.add_node(element->guid(), "element_" + element->name);
  auto node = std::make_shared<TreeNode>(element->guid());
  if (parent) add(node, parent);
  return node;
}

std::shared_ptr<TreeNode> Session::add_component(Component component, std::shared_ptr<TreeNode> parent) {
  component_lookup[component.guid()] = component;
  objects.components->push_back(component);
  graph.add_node(component.guid(), "component_" + component.name);
  auto node = std::make_shared<TreeNode>(component.guid());
  if (parent) add(node, parent);
  return node;
}

std::shared_ptr<TreeNode> Session::add_group(const std::string& group_name) {
  auto node = std::make_shared<TreeNode>(group_name);
  add(node);
  return node;
}

std::shared_ptr<TreeNode> Session::find_group(const std::string& group_name) const {
  auto r = tree.root();
  if (r) {
    for (auto* child : r->children()) {
      if (child && child->name == group_name) {
        return child->shared_from_this();
      }
    }
  }
  throw std::runtime_error("Group '" + group_name + "' not found");
}

void Session::add(std::shared_ptr<TreeNode> node,
                  std::shared_ptr<TreeNode> parent) {
  if (parent == nullptr) {
    tree.add(node, tree.root());
  } else {
    tree.add(node, parent);
  }
}

void Session::add_edge(const std::string &guid1, const std::string &guid2,
                       const std::string &attribute) {
  graph.add_edge(guid1, guid2, attribute);
}

std::vector<std::string> Session::order() const {
  std::vector<std::string> order;
  order.reserve(lookup.size());
  for (const auto &p : *objects.points) order.push_back(p->guid());
  for (const auto &l : *objects.lines) order.push_back(l->guid());
  for (const auto &p : *objects.planes) order.push_back(p->guid());
  for (const auto &b : *objects.bboxes) order.push_back(b->guid());
  for (const auto &p : *objects.polylines) order.push_back(p->guid());
  for (const auto &p : *objects.pointclouds) order.push_back(p->guid());
  for (const auto &m : *objects.meshes) order.push_back(m->guid());
  for (const auto &n : *objects.nurbscurves) order.push_back(n->guid());
  for (const auto &n : *objects.nurbssurfaces) order.push_back(n->guid());
  for (const auto &b : *objects.breps) order.push_back(b->guid());
  for (const auto &e : *objects.elements) order.push_back(e->guid());
  return order;
}

// ═══════════════════════════════════════════════════════════════════════════
// Xforms - the one place a transformation is stored
// ═══════════════════════════════════════════════════════════════════════════

void Session::set_xform(const std::string &guid, const Xform &xform) {
  xforms[guid] = xform;
  bvh_cache_dirty = true;
}

Xform Session::xform(const std::string &guid) const {
  auto it = xforms.find(guid);
  return it == xforms.end() ? Xform::identity() : it->second;
}

bool Session::remove_xform(const std::string &guid) {
  bool removed = xforms.erase(guid) > 0;
  if (removed) {
    bvh_cache_dirty = true;
  }
  return removed;
}

Xform Session::world_xform(const std::string &guid) const {
  Xform acc = xform(guid);
  auto node = tree.get_node_by_name(guid);
  if (node) {
    // ancestors() runs immediate parent -> root, so left-multiplying each in turn yields
    // root * ... * parent * local - the same order the tree walk composes.
    for (auto *ancestor : node->ancestors()) {
      auto it = xforms.find(ancestor->name);
      if (it != xforms.end()) {
        acc = it->second * acc;
      }
    }
  }
  // No tree node: the object is its own root, so acc stays its local transform. Returning
  // identity here would silently move every unparented object to the origin.
  return acc;
}

std::unordered_map<std::string, Xform> Session::world_xforms() const {
  // Nothing to compose: with no local transforms every composed frame IS the identity, and
  // every caller already falls back to identity for a guid the map lacks. Walking the tree
  // anyway costs one string copy + hash insert per NODE, paid again on every rebuild.
  if (xforms.empty()) {
    return {};
  }

  std::unordered_map<std::string, Xform> out;
  std::function<void(const std::shared_ptr<TreeNode> &, const Xform &)> walk =
      [&](const std::shared_ptr<TreeNode> &node, const Xform &parent_xform) {
        auto it = xforms.find(node->name);
        Xform current = it == xforms.end() ? parent_xform : parent_xform * it->second;
        out[node->name] = current;
        for (auto *child : node->children()) {
          walk(child->shared_from_this(), current);
        }
      };

  if (tree.root()) {
    walk(tree.root(), Xform::identity());
  }
  // Objects that were added without a parent have no tree node; they are their own roots.
  for (const auto &[obj_guid, obj_xform] : xforms) {
    out.emplace(obj_guid, obj_xform);
  }
  return out;
}

std::vector<std::pair<std::string, Xform>> Session::xforms_ordered() const {
  std::vector<std::pair<std::string, Xform>> ordered;
  for (const auto &g : order()) {
    auto it = xforms.find(g);
    if (it != xforms.end() && !it->second.is_identity()) {
      ordered.emplace_back(g, it->second);
    }
  }
  // Group nodes carry transforms too but hold no geometry, so they are absent from order();
  // they follow, sorted by guid, or a group's placement would be lost on save.
  std::set<std::string> listed;
  for (const auto &[g, x] : ordered) listed.insert(g);
  std::vector<std::pair<std::string, Xform>> rest;
  for (const auto &[obj_guid, obj_xform] : xforms) {
    if (!listed.count(obj_guid) && !obj_xform.is_identity()) {
      rest.emplace_back(obj_guid, obj_xform);
    }
  }
  std::sort(rest.begin(), rest.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });
  ordered.insert(ordered.end(), rest.begin(), rest.end());
  return ordered;
}

bool Session::remove_object(const std::string &obj_guid) {
  bvh_cache_dirty = true; // removed objects must vanish from the ray BVH
  auto it = lookup.find(obj_guid);
  if (it == lookup.end()) {
    return false;
  }

  invalidate_bvh_cache();

  // Determine type and remove from typed collection
  std::visit(
      [this](const auto &ptr) {
        using T = std::decay_t<decltype(*ptr)>;
        auto erase_by_guid = [&](auto &vec) {
          vec.erase(std::remove_if(
                        vec.begin(), vec.end(),
                        [&](const auto &p) { return p->guid() == ptr->guid(); }),
                    vec.end());
        };
        if constexpr (std::is_same_v<T, Point>) {
          erase_by_guid(*objects.points);
        }
        else if constexpr (std::is_same_v<T, Line>) {
          erase_by_guid(*objects.lines);
        }
        else if constexpr (std::is_same_v<T, Plane>) {
          erase_by_guid(*objects.planes);
        }
        else if constexpr (std::is_same_v<T, OBB>) {
          erase_by_guid(*objects.bboxes);
        }
        else if constexpr (std::is_same_v<T, Polyline>) {
          erase_by_guid(*objects.polylines);
        }
        else if constexpr (std::is_same_v<T, PointCloud>) {
          erase_by_guid(*objects.pointclouds);
        }
        else if constexpr (std::is_same_v<T, Mesh>) {
          erase_by_guid(*objects.meshes);
        }
        else if constexpr (std::is_same_v<T, NurbsCurve>) {
          erase_by_guid(*objects.nurbscurves);
        }
        else if constexpr (std::is_same_v<T, NurbsSurface>) {
          erase_by_guid(*objects.nurbssurfaces);
        }
        else if constexpr (std::is_same_v<T, BRep>) {
          erase_by_guid(*objects.breps);
        }
        else if constexpr (std::is_same_v<T, Element>) {
          erase_by_guid(*objects.elements);
        }
      },
      it->second);

  // Remove from lookup table
  lookup.erase(it);
  xforms.erase(obj_guid);

  // Remove from tree
  auto tree_node = tree.find_node_by_guid(obj_guid);
  if (tree_node) {
    tree.remove(tree_node);
  }

  // Remove from graph
  if (graph.has_node(obj_guid)) {
    graph.remove_node(obj_guid);
  }

  return true;
}

// Tree Operations

bool Session::add_hierarchy(const std::string &parent_guid,
                            const std::string &child_guid) {
  return tree.add_child_by_guid(parent_guid, child_guid);
}

std::vector<std::string> Session::get_children(const std::string &obj_guid) const {
  return tree.get_children_guids(obj_guid);
}

// Graph Operations

void Session::add_relationship(const std::string &from_guid,
                               const std::string &to_guid,
                               const std::string &relationship_type) {
  graph.add_edge(from_guid, to_guid, relationship_type);
}

std::vector<std::string> Session::get_neighbours(const std::string &obj_guid) {
  return graph.neighbors(obj_guid);
}

// SpatialBVH Collision Detection

// ADD NEW GEOMETRY TYPES HERE: Add bounding box computation for collision detection
OBB Session::compute_bounding_box(const Geometry& geometry, const Xform& xform) {
  double inflate = Tolerance::APPROXIMATION;
  auto tp = [&xform](const Point& p) -> Point { return xform.transform_point(p); };

  return std::visit([inflate, &xform, &tp](auto&& geom_ptr) -> OBB {
    using T = std::decay_t<decltype(geom_ptr)>;

    if constexpr (std::is_same_v<T, std::shared_ptr<Point>>) {
      return OBB::from_point(tp(*geom_ptr), inflate);
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<Line>>) {
      std::vector<Point> points = {tp(geom_ptr->start()), tp(geom_ptr->end())};
      return OBB::from_points(points, inflate);
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<Polyline>>) {
      std::vector<Point> points;
      for (const auto& p : geom_ptr->get_points()) points.push_back(tp(p));
      return OBB::from_points(points, inflate);
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<PointCloud>>) {
      std::vector<Point> points;
      for (const auto& p : geom_ptr->get_points()) points.push_back(tp(p));
      return OBB::from_points(points, inflate);
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<Mesh>>) {
      std::vector<Point> points;
      for (const auto& [key, vertex] : geom_ptr->vertex) {
        points.push_back(tp(vertex.position()));
      }
      if (points.empty()) {
        return OBB::from_point(Point(0, 0, 0), inflate);
      }
      return OBB::from_points(points, inflate);
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<BRep>>) {
      std::vector<Point> points;
      for (const auto& v : geom_ptr->m_vertices) {
        points.push_back(tp(v.point));
      }
      // Sample surface points to cover curved surfaces (e.g. sphere with only pole vertices)
      for (const auto& srf : geom_ptr->m_surfaces) {
        auto [u0, u1] = srf.domain(0);
        auto [v0, v1] = srf.domain(1);
        for (int ui = 0; ui <= 2; ++ui) {
          for (int vi = 0; vi <= 2; ++vi) {
            double u = u0 + (u1 - u0) * ui / 2.0;
            double v = v0 + (v1 - v0) * vi / 2.0;
            points.push_back(tp(srf.point_at(u, v)));
          }
        }
      }
      if (points.empty()) {
        return OBB::from_point(Point(0, 0, 0), inflate);
      }
      return OBB::from_points(points, inflate);
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<OBB>>) {
      // Inflate existing bounding box
      auto inflated = *geom_ptr;
      inflated.half_size = Vector(
        inflated.half_size[0] + inflate,
        inflated.half_size[1] + inflate,
        inflated.half_size[2] + inflate
      );
      inflated.transform(xform);
      return inflated;
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<Plane>>) {
      // Create bounded box around plane origin
      return OBB::from_point(tp(geom_ptr->origin()), inflate * 10.0);
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<NurbsCurve>>) {
      std::vector<Point> points;
      for (int i = 0; i < geom_ptr->cv_count(); ++i)
        points.push_back(tp(geom_ptr->get_cv(i)));
      if (points.empty()) return OBB::from_point(Point(0, 0, 0), inflate);
      return OBB::from_points(points, inflate);
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<NurbsSurface>>) {
      std::vector<Point> points;
      for (int i = 0; i < geom_ptr->cv_count(0); ++i)
        for (int j = 0; j < geom_ptr->cv_count(1); ++j)
          points.push_back(tp(geom_ptr->get_cv(i, j)));
      if (points.empty()) return OBB::from_point(Point(0, 0, 0), inflate);
      return OBB::from_points(points, inflate);
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<Element>>) {
      auto e_copy = *geom_ptr;
      auto box = e_copy.aabb();
      box.transform(xform);
      return box;
    }
    else {
      return OBB::from_point(Point(0, 0, 0), inflate);
    }
  }, geometry);
}

std::vector<std::pair<std::string, std::string>> Session::get_collisions() {
  // Collect all objects with their bounding boxes and GUIDs
  std::vector<OBB> boxes;
  std::vector<std::string> guids;
  boxes.reserve(lookup.size());
  guids.reserve(lookup.size());

  auto world = world_xforms();
  for (const auto& g : order()) {
    auto it = lookup.find(g);
    if (it == lookup.end()) continue;
    auto wit = world.find(g);
    boxes.push_back(compute_bounding_box(it->second, wit == world.end() ? Xform::identity() : wit->second));
    guids.push_back(g);
  }
  
  if (boxes.empty()) {
    return {};
  }
  
  // Build SpatialBVH and check collisions
  double world_size = SpatialBVH::compute_world_size(boxes);
  bvh = SpatialBVH::from_boxes(boxes, world_size);
  auto [collision_pairs, colliding_indices, checks] = bvh.check_all_collisions(boxes);
  (void)colliding_indices;
  (void)checks;
  
  // Map index pairs to GUID pairs
  std::vector<std::pair<std::string, std::string>> guid_pairs;
  guid_pairs.reserve(collision_pairs.size());
  for (const auto& [i, j] : collision_pairs) {
    if (i >= 0 && i < static_cast<int>(guids.size()) && 
        j >= 0 && j < static_cast<int>(guids.size())) {
      guid_pairs.emplace_back(guids[i], guids[j]);
    }
  }
  
  // Add collision edges to graph
  for (const auto& [guid1, guid2] : guid_pairs) {
    graph.add_edge(guid1, guid2, "bvh_collision");
  }
  
  return guid_pairs;
}

// Transformed Geometry

Objects Session::get_geometry() const {
  // A REAL deep copy. Objects holds shared_ptr<vector<shared_ptr<T>>> and has no copy
  // constructor, so `Objects copy = objects;` shared both the vectors and the objects
  // themselves - this const method used to mutate the session's own geometry.
  Objects out(objects.name);
  out.guid() = objects.guid();

  auto clone_into = [](const auto& src_vec, auto& dst_vec) {
    for (const auto& src : src_vec) {
      using T = typename std::decay_t<decltype(*src)>;
      auto copy = std::make_shared<T>(*src);
      copy->guid() = src->guid();
      dst_vec.push_back(copy);
    }
  };
  clone_into(*objects.points, *out.points);
  clone_into(*objects.lines, *out.lines);
  clone_into(*objects.planes, *out.planes);
  clone_into(*objects.bboxes, *out.bboxes);
  clone_into(*objects.polylines, *out.polylines);
  clone_into(*objects.pointclouds, *out.pointclouds);
  clone_into(*objects.meshes, *out.meshes);
  clone_into(*objects.nurbscurves, *out.nurbscurves);
  clone_into(*objects.nurbssurfaces, *out.nurbssurfaces);
  clone_into(*objects.breps, *out.breps);
  // Element's copy constructor mints a fresh guid, so identity is restored after the copy.
  for (const auto& src : *objects.elements) {
    auto copy = std::make_shared<Element>(*src);
    copy->guid() = src->guid();
    out.elements->push_back(copy);
  }
  *out.components = *objects.components;

  auto world = world_xforms();

  // No type is skipped: C++ used to leave breps un-baked, so a placement in the tree silently
  // did nothing for them.
  auto bake = [&world](auto& vec) {
    for (auto& item : vec) {
      auto it = world.find(item->guid());
      if (it == world.end() || it->second.is_identity()) continue;
      item->transform(it->second);
    }
  };
  bake(*out.points);
  bake(*out.lines);
  bake(*out.planes);
  bake(*out.bboxes);
  bake(*out.polylines);
  bake(*out.pointclouds);
  bake(*out.meshes);
  bake(*out.nurbscurves);
  bake(*out.nurbssurfaces);
  bake(*out.breps);
  for (auto& item : *out.elements) {
    auto it = world.find(item->guid());
    if (it == world.end() || it->second.is_identity()) continue;
    item->place(it->second);
  }

  return out;
}

// JSON Serialization

nlohmann::ordered_json Session::jsondump() const {
  nlohmann::ordered_json data;
  data["type"] = "Session";
  data["name"] = name;
  data["guid"] = guid();
  data["objects"] = objects.jsondump();
  data["tree"] = tree.jsondump();
  data["graph"] = graph.jsondump();
  nlohmann::ordered_json xforms_json = nlohmann::ordered_json::array();
  for (const auto &[obj_guid, obj_xform] : xforms_ordered()) {
    nlohmann::ordered_json entry;
    entry["guid"] = obj_guid;
    entry["xform"] = obj_xform.jsondump();
    xforms_json.push_back(entry);
  }
  data["xforms"] = xforms_json;
  return data;
}

Session Session::jsonload(const nlohmann::json &data) {
  Session session(data.value("name", "my_session"));

  // Load objects
  if (data.contains("objects")) {
    session.objects = Objects::jsonload(data["objects"]);
  }

  // Rebuild lookup from all objects
  for (const auto &bbox_ptr : *session.objects.bboxes) {
    session.lookup[bbox_ptr->guid()] = bbox_ptr;
  }
  for (const auto &line_ptr : *session.objects.lines) {
    session.lookup[line_ptr->guid()] = line_ptr;
  }
  for (const auto &mesh_ptr : *session.objects.meshes) {
    session.lookup[mesh_ptr->guid()] = mesh_ptr;
  }
  for (const auto &nc_ptr : *session.objects.nurbscurves) {
    session.lookup[nc_ptr->guid()] = nc_ptr;
  }
  for (const auto &ns_ptr : *session.objects.nurbssurfaces) {
    session.lookup[ns_ptr->guid()] = ns_ptr;
  }
  for (const auto &plane_ptr : *session.objects.planes) {
    session.lookup[plane_ptr->guid()] = plane_ptr;
  }
  for (const auto &point_ptr : *session.objects.points) {
    session.lookup[point_ptr->guid()] = point_ptr;
  }
  for (const auto &pointcloud_ptr : *session.objects.pointclouds) {
    session.lookup[pointcloud_ptr->guid()] = pointcloud_ptr;
  }
  for (const auto &polyline_ptr : *session.objects.polylines) {
    session.lookup[polyline_ptr->guid()] = polyline_ptr;
  }
  for (const auto &brep_ptr : *session.objects.breps) {
    session.lookup[brep_ptr->guid()] = brep_ptr;
  }
  for (const auto &element_ptr : *session.objects.elements) {
    session.lookup[element_ptr->guid()] = element_ptr;
  }

  // Load tree structure
  if (data.contains("tree")) {
    session.tree = Tree::jsonload(data["tree"]);
  }

  // Load graph structure
  if (data.contains("graph")) {
    session.graph = Graph::jsonload(data["graph"]);
  }

  // Load local transforms (absent in older files)
  if (data.contains("xforms")) {
    for (const auto &entry : data["xforms"]) {
      session.xforms[entry["guid"].get<std::string>()] = Xform::jsonload(entry["xform"]);
    }
  }


  return session;
}

std::string Session::file_json_dumps() const {
  return jsondump().dump();
}

Session Session::file_json_loads(const std::string& json_string) {
  return jsonload(nlohmann::ordered_json::parse(json_string));
}

void Session::file_json_dump(const std::string& filename) const {
  std::ofstream file(filename);
  file << jsondump().dump(4);
}

Session Session::file_json_load(const std::string& filename) {
  std::ifstream file(filename);
  nlohmann::json data = nlohmann::json::parse(file);
  return jsonload(data);
}

std::string Session::pb_dumps() const {
  session_proto::Session proto;
  proto.set_name(name);
  proto.set_guid(guid());
  proto.mutable_objects()->ParseFromString(objects.pb_dumps());
  proto.mutable_tree()->ParseFromString(tree.pb_dumps());
  proto.mutable_graph()->ParseFromString(graph.pb_dumps());
  // Xforms in canonical order() sequence - a map would not be deterministic
  for (const auto &[obj_guid, obj_xform] : xforms_ordered()) {
    auto *entry = proto.add_xforms();
    entry->set_guid(obj_guid);
    entry->mutable_xform()->ParseFromString(obj_xform.pb_dumps());
  }
  return proto.SerializeAsString();
}

Session Session::pb_loads(const std::string& data) {
  session_proto::Session proto;
  proto.ParseFromString(data);

  Session session(proto.name());
  session.guid() = proto.guid();

  if (proto.has_objects()) {
    session.objects = Objects::pb_loads(proto.objects().SerializeAsString());
  }
  if (proto.has_tree()) {
    session.tree = Tree::pb_loads(proto.tree().SerializeAsString());
  }
  if (proto.has_graph()) {
    session.graph = Graph::pb_loads(proto.graph().SerializeAsString());
  }


  for (const auto& p : *session.objects.points) session.lookup[p->guid()] = p;
  for (const auto& l : *session.objects.lines) session.lookup[l->guid()] = l;
  for (const auto& pl : *session.objects.planes) session.lookup[pl->guid()] = pl;
  for (const auto& b : *session.objects.bboxes) session.lookup[b->guid()] = b;
  for (const auto& pl : *session.objects.polylines) session.lookup[pl->guid()] = pl;
  for (const auto& pc : *session.objects.pointclouds) session.lookup[pc->guid()] = pc;
  for (const auto& m : *session.objects.meshes) session.lookup[m->guid()] = m;
  for (const auto& nc : *session.objects.nurbscurves) session.lookup[nc->guid()] = nc;
  for (const auto& ns : *session.objects.nurbssurfaces) session.lookup[ns->guid()] = ns;
  for (const auto& b : *session.objects.breps) session.lookup[b->guid()] = b;
  for (const auto& e : *session.objects.elements) session.lookup[e->guid()] = e;

  for (const auto& entry : proto.xforms()) {
    session.xforms[entry.guid()] = Xform::pb_loads(entry.xform().SerializeAsString());
  }

  return session;
}

void Session::pb_dump(const std::string& filename) const {
  std::string data = pb_dumps();
  std::ofstream file(filename, std::ios::binary);
  file.write(data.data(), data.size());
}

Session Session::pb_load(const std::string& filename) {
  std::ifstream file(filename, std::ios::binary);
  std::string data((std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());
  return pb_loads(data);
}

// Ray Intersection

void Session::cache_geometry_aabb(const std::string& obj_guid, const Geometry& geometry) {
  // LAZY (P5): boxes are recomputed in rebuild_ray_bvh_cache from the canonical order —
  // always fresh, nothing retained per-add. Adds only mark the cache dirty.
  (void)obj_guid; (void)geometry;
  bvh_cache_dirty = true;
}

void Session::rebuild_ray_bvh_cache() {
  // LAZY (P5): boxes are recomputed here from the document in canonical order() — always
  // fresh (an object mutated through lookup gets a fresh box), deterministic across runs
  // and languages, and LOCAL: the BVH copies them into its nodes, so nothing box-shaped is
  // retained on Session (cached_boxes stays empty; kept only for API compatibility).
  cached_boxes.clear();
  cached_guids.clear();
  std::vector<OBB> boxes;
  boxes.reserve(lookup.size());
  auto world = world_xforms();
  for (const auto& g : order()) {
    auto it = lookup.find(g);
    if (it == lookup.end()) continue;
    auto wit = world.find(g);
    boxes.push_back(compute_bounding_box(it->second, wit == world.end() ? Xform::identity() : wit->second));
    cached_guids.push_back(g);
  }
  if (!boxes.empty()) {
    double world_size = SpatialBVH::compute_world_size(boxes);
    cached_ray_bvh = SpatialBVH::from_boxes(boxes, world_size);
  } else {
    cached_ray_bvh = SpatialBVH();
  }
}

std::vector<Session::RayHit> Session::ray_cast(const Point& origin, const Vector& direction, double tolerance) {
  // Rebuild SpatialBVH cache if dirty (geometry added/removed)
  if (bvh_cache_dirty) {
    rebuild_ray_bvh_cache();
    bvh_cache_dirty = false;
  }
  
  if (cached_guids.empty()) return {};
  
  // SpatialBVH OPTIMIZATION: Get candidate indices from CACHED SpatialBVH ray traversal
  // This prunes objects whose AABBs don't intersect the ray, providing
  // acceleration for ALL geometry types (Point, Line, Mesh, etc.)
  std::vector<int> candidate_ids;
  cached_ray_bvh.ray_cast(origin, direction, candidate_ids, true);
  
  // Test candidates with precise geometry intersection and track closest hit
  std::vector<RayHit> hits;
  Line ray = Line::from_points(origin, origin + direction * 10000.0);  // Long ray
  double closest_dist = std::numeric_limits<double>::infinity();

  // Placements come from the session, not the geometry; resolve them all up front.
  auto world = world_xforms();

  for (int idx : candidate_ids) {
    const std::string& obj_guid = cached_guids[idx];
    auto lookup_it = lookup.find(obj_guid);
    if (lookup_it == lookup.end()) continue; // removed since the BVH was built
    const Geometry& geom = lookup_it->second;
    auto wit = world.find(obj_guid);
    Xform placement = wit == world.end() ? Xform::identity() : wit->second;

    std::optional<Point> hit = ray_intersect_geometry(ray, geom, tolerance, placement);
    if (hit) {
      double dist = origin.distance(*hit);
      
      // Only keep hits closer than current closest
      if (dist < closest_dist) {
        // Clear previous hits if this is closer
        if (dist < closest_dist - tolerance) {
          hits.clear();
        }
        hits.push_back({obj_guid, *hit, dist});
        closest_dist = dist;
      }
    }
  }
  
  // Already sorted by discovery order (closest first)
  return hits;
}

// ADD NEW GEOMETRY TYPES HERE: Add ray intersection logic for precise ray casting
std::optional<Point> Session::ray_intersect_geometry(const Line& ray, const Geometry& geometry, double tolerance, const Xform& placement) {
  return std::visit([&](auto&& geom_ptr) -> std::optional<Point> {
    using T = std::decay_t<decltype(geom_ptr)>;
    
    if constexpr (std::is_same_v<T, std::shared_ptr<Point>>) {
      // Ray-point: parametric distance calculation
      Vector ray_dir = ray.end() - ray.start();
      Vector to_point = *geom_ptr - ray.start();
      double t = to_point.dot(ray_dir) / ray_dir.dot(ray_dir);
      if (t < 0) return std::nullopt;  // Point is behind ray
      Point closest = ray.start() + ray_dir * t;
      double dist = geom_ptr->distance(closest);
      return (dist <= tolerance) ? std::optional{closest} : std::nullopt;
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<Line>>) {
      // Ray-line intersection
      Point hit;
      if (Intersection::line_line(ray, *geom_ptr, hit, tolerance)) {
        return hit;
      }
      return std::nullopt;
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<Plane>>) {
      // Ray-plane intersection
      Point hit;
      if (Intersection::line_plane(ray, *geom_ptr, hit, tolerance)) {
        return hit;
      }
      return std::nullopt;
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<Polyline>>) {
      // Ray-polyline: test each segment, return closest
      std::optional<Point> closest_hit;
      double min_dist = std::numeric_limits<double>::infinity();
      
      for (size_t i = 0; i < geom_ptr->segment_count(); ++i) {
        Point p0 = geom_ptr->get_point(i);
        Point p1 = geom_ptr->get_point(i + 1);
        Line seg = Line::from_points(p0, p1);
        Point hit;
        if (Intersection::line_line(ray, seg, hit, tolerance)) {
          double dist = ray.start().distance(hit);
          if (dist < min_dist) {
            min_dist = dist;
            closest_hit = hit;
          }
        }
      }
      return closest_hit;
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<PointCloud>>) {
      // Ray-pointcloud: find closest point within tolerance
      std::optional<Point> closest_hit;
      double min_dist = std::numeric_limits<double>::infinity();
      Vector ray_dir = ray.end() - ray.start();
      
      for (const Point& pt : geom_ptr->get_points()) {
        Vector to_point = pt - ray.start();
        double t = to_point.dot(ray_dir) / ray_dir.dot(ray_dir);
        if (t < 0) continue;  // Point is behind ray
        Point closest = ray.start() + ray_dir * t;
        double dist = pt.distance(closest);
        if (dist <= tolerance && dist < min_dist) {
          min_dist = dist;
          closest_hit = closest;
        }
      }
      return closest_hit;
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<Mesh>>) {
      // The session holds the placement: cast in the mesh's LOCAL frame, return a WORLD hit
      auto inv = placement.inverse();
      if (!inv) return std::nullopt;
      Line local_ray = Line::from_points(inv->transform_point(ray.start()), inv->transform_point(ray.end()));
      std::vector<Point> hits = Intersection::ray_mesh_bvh(local_ray, *geom_ptr, tolerance, true);
      if (!hits.empty()) {
        return placement.transform_point(hits[0]);  // Return closest
      }
      return std::nullopt;
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<OBB>>) {
      // Ray-bbox: use slab test to find entry point
      double tmin, tmax;
      if (Intersection::ray_box(ray.start(), ray.end() - ray.start(), *geom_ptr, 0.0, 1.0, tmin, tmax)) {
        // Return entry point (closest intersection)
        Vector ray_dir = ray.end() - ray.start();
        Point hit = ray.start() + ray_dir * tmin;
        return hit;
      }
      return std::nullopt;
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<NurbsCurve>>) {
      return std::nullopt;
    }
    else if constexpr (std::is_same_v<T, std::shared_ptr<NurbsSurface>>) {
      return std::nullopt;
    }
    else {
      return std::nullopt;
    }
  }, geometry);
}

std::ostream &operator<<(std::ostream &os, const Session &session) {
  return os << session.str();
}

} // namespace session_cpp