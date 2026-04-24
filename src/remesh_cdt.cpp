#include "remesh_cdt.h"
#include "session_config.h"
#include <vector>
#include <array>
#include <map>
#include <set>
#include <stack>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace session_cpp {
namespace {

// ---- Types ----

// Hash for pair<int64_t,int64_t> used in unordered_map.
struct P64Hash {
    size_t operator()(const std::pair<int64_t,int64_t>& p) const noexcept {
        size_t h = std::hash<int64_t>()(p.first);
        h ^= std::hash<int64_t>()(p.second) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        return h;
    }
};

// Integer point scaled by 1e6 for exact arithmetic during sweep.
struct Point64 {
    int64_t x = 0, y = 0;
    bool operator==(const Point64& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Point64& o) const { return !(*this == o); }
};
using Path64  = std::vector<Point64>;
using Paths64 = std::vector<Path64>;

enum class TriangulateResult { success, fail, no_polygons, paths_intersect };
// Boundary edges are ascend (left boundary, going up-left) or descend (right boundary,
// going up-right). Interior diagonals created during triangulation are loose.
enum class EdgeKind           { loose, ascend, descend };
enum class IntersectKind      { none, collinear, intersect };
enum class EdgeContainsResult { neither, left, right };

// ---- Math helpers ----

// Sign of the 2-D cross product at p2 on the turn p1→p2→p3.
inline int CrossProductSign(const Point64& p1, const Point64& p2, const Point64& p3)
{
    double cp = static_cast<double>(p2.x - p1.x) * static_cast<double>(p3.y - p2.y) -
                static_cast<double>(p2.y - p1.y) * static_cast<double>(p3.x - p2.x);
    if (cp > 0) return 1;
    if (cp < 0) return -1;
    return 0;
}

inline double Sqr(double x) { return x * x; }

inline double DistanceSqr(const Point64& a, const Point64& b)
{
    return Sqr(static_cast<double>(a.x - b.x)) + Sqr(static_cast<double>(a.y - b.y));
}

inline void DoError(int) { throw std::runtime_error("cdt error"); }

// ---- Forward declarations ----

class Vertex2;
class Edge;
class CdtTri;

using VertexList = std::vector<Vertex2*>;
using EdgeList   = std::vector<Edge*>;

// ---- Graph nodes ----

class Vertex2 {
public:
    Point64  pt;
    EdgeList edges;
    bool     innerLM = false;   // true for hole local minima
    Vertex2(const Point64& p64) : pt(p64) { edges.reserve(2); }
};

// Each edge stores left/right and bottom/top endpoint pointers,
// its classification, and pointers to at most two incident triangles.
class Edge {
public:
    Vertex2*  vL = nullptr;
    Vertex2*  vR = nullptr;
    Vertex2*  vB = nullptr;
    Vertex2*  vT = nullptr;
    EdgeKind  kind = EdgeKind::loose;
    CdtTri*   triA = nullptr;
    CdtTri*   triB = nullptr;
    bool      isActive = false;
    Edge*     nextE = nullptr;  // doubly-linked active-edge list
    Edge*     prevE = nullptr;
};

// Triangle owns three edge pointers (may be boundary or interior).
class CdtTri {
public:
    Edge* edges[3];
    CdtTri(Edge* e1, Edge* e2, Edge* e3) { edges[0] = e1; edges[1] = e2; edges[2] = e3; }
};

// ---- Sorting predicates ----

static bool VertexListSort(const Vertex2* a, const Vertex2* b)
{
    return (a->pt.y == b->pt.y) ? (a->pt.x < b->pt.x) : (a->pt.y > b->pt.y);
}

static bool EdgeListSort(const Edge* a, const Edge* b)
{
    return a->vL->pt.x < b->vL->pt.x;
}

// ---- Edge predicates ----

static bool IsLooseEdge(const Edge& e) { return e.kind == EdgeKind::loose; }
static bool IsLeftEdge(const Edge& e)  { return e.kind == EdgeKind::ascend; }
static bool IsRightEdge(const Edge& e) { return e.kind == EdgeKind::descend; }
static bool IsHorizontal(const Edge& e){ return e.vB->pt.y == e.vT->pt.y; }

static bool LeftTurning(const Point64& p1, const Point64& p2, const Point64& p3)
{
    return CrossProductSign(p1, p2, p3) < 0;
}

static bool RightTurning(const Point64& p1, const Point64& p2, const Point64& p3)
{
    return CrossProductSign(p1, p2, p3) > 0;
}

// An edge is done when it has two triangles, or one triangle and is not loose.
static bool EdgeCompleted(Edge* edge)
{
    if (!edge->triA) return false;
    if (edge->triB)  return true;
    return edge->kind != EdgeKind::loose;
}

static EdgeContainsResult EdgeContains(const Edge* edge, const Vertex2* v)
{
    if (edge->vL == v) return EdgeContainsResult::left;
    if (edge->vR == v) return EdgeContainsResult::right;
    return EdgeContainsResult::neither;
}


static void RemoveEdgeFromVertex(Vertex2* vert, Edge* edge)
{
    auto it = std::find(vert->edges.begin(), vert->edges.end(), edge);
    if (it == vert->edges.end()) DoError(0);
    vert->edges.erase(it);
}

// Walk the path forward from idx to find a proper local minimum (lowest point
// between a descending and an ascending run).
static bool FindLocMinIdx(const Path64& path, size_t len, size_t& idx)
{
    if (len < 3) return false;
    size_t i0 = idx, n = (idx + 1) % len;
    while (path[n].y <= path[idx].y) {
        idx = n; n = (n + 1) % len;
        if (idx == i0) return false;
    }
    while (path[n].y >= path[idx].y) {
        idx = n; n = (n + 1) % len;
    }
    return true;
}

static size_t Prev(size_t& idx, size_t len)
{
    return (idx == 0) ? len - 1 : idx - 1;
}

static size_t Next(size_t& idx, size_t len)
{
    return (idx + 1) % len;
}

// Find an existing edge between two vertices; prefer the kind indicated by preferAscending.
static Edge* FindLinkingEdge(const Vertex2* vert1, const Vertex2* vert2, bool preferAscending)
{
    Edge* res = nullptr;
    for (auto e : vert1->edges) {
        if (e->vL == vert2 || e->vR == vert2) {
            if (e->kind == EdgeKind::loose ||
                ((e->kind == EdgeKind::ascend) == preferAscending)) return e;
            res = e;
        }
    }
    return res;
}

// Extract the three CCW points from a triangle (third vertex deduced from edges[1]).
static Path64 PathFromTriangle(CdtTri tri)
{
    Path64 res;
    res.reserve(3);
    res.push_back(tri.edges[0]->vL->pt);
    res.push_back(tri.edges[0]->vR->pt);
    const Edge& e = *tri.edges[1];
    if (e.vL->pt == res[0] || e.vL->pt == res[1])
        res.push_back(e.vR->pt);
    else
        res.push_back(e.vL->pt);
    return res;
}

// Delaunay in-circle test: returns > 0 if D lies inside the circumcircle of CCW triangle ABC.
static double InCircleTest(const Point64& ptA, const Point64& ptB,
    const Point64& ptC, const Point64& ptD)
{
    double m00 = static_cast<double>(ptA.x - ptD.x);
    double m01 = static_cast<double>(ptA.y - ptD.y);
    double m02 = Sqr(m00) + Sqr(m01);
    double m10 = static_cast<double>(ptB.x - ptD.x);
    double m11 = static_cast<double>(ptB.y - ptD.y);
    double m12 = Sqr(m10) + Sqr(m11);
    double m20 = static_cast<double>(ptC.x - ptD.x);
    double m21 = static_cast<double>(ptC.y - ptD.y);
    double m22 = Sqr(m20) + Sqr(m21);
    return m00 * (m11 * m22 - m21 * m12) -
           m10 * (m01 * m22 - m21 * m02) +
           m20 * (m01 * m12 - m11 * m02);
}

// Squared shortest distance from pt to the segment [segPt1, segPt2].
static double ShortestDistFromSegment(const Point64& pt,
    const Point64& segPt1, const Point64& segPt2)
{
    double dx = static_cast<double>(segPt2.x - segPt1.x);
    double dy = static_cast<double>(segPt2.y - segPt1.y);
    double ax = static_cast<double>(pt.x - segPt1.x);
    double ay = static_cast<double>(pt.y - segPt1.y);
    double qNum = ax * dx + ay * dy;
    if (qNum < 0)                  return DistanceSqr(pt, segPt1);
    if (qNum > Sqr(dx) + Sqr(dy)) return DistanceSqr(pt, segPt2);
    return Sqr(ax * dy - dx * ay) / (dx * dx + dy * dy);
}

// Strict interior segment intersection (shared endpoints → none).
static IntersectKind SegsIntersect(const Point64 s1a, const Point64 s1b,
    const Point64 s2a, const Point64 s2b)
{
    if (s1a == s2a || s1b == s2a || s1b == s2b) return IntersectKind::none;
    double dy1 = static_cast<double>(s1b.y - s1a.y);
    double dx1 = static_cast<double>(s1b.x - s1a.x);
    double dy2 = static_cast<double>(s2b.y - s2a.y);
    double dx2 = static_cast<double>(s2b.x - s2a.x);
    double cp = dy1 * dx2 - dy2 * dx1;
    if (cp == 0) return IntersectKind::collinear;
    double t = static_cast<double>(s1a.x - s2a.x) * dy2 -
               static_cast<double>(s1a.y - s2a.y) * dx2;
    if (t >= 0) { if (cp < 0 || t >= cp) return IntersectKind::none; }
    else        { if (cp > 0 || t <= cp) return IntersectKind::none; }
    t = static_cast<double>(s1a.x - s2a.x) * dy1 -
        static_cast<double>(s1a.y - s2a.y) * dx1;
    if (t >= 0) { if (cp > 0 && t < cp) return IntersectKind::intersect; }
    else        { if (cp < 0 && t > cp) return IntersectKind::intersect; }
    return IntersectKind::none;
}

static double DistSqr(const Point64& pt1, const Point64& pt2)
{
    return Sqr(static_cast<double>(pt1.x - pt2.x)) + Sqr(static_cast<double>(pt1.y - pt2.y));
}

// ---- Sweep-line CDT engine ----

class Delaunay {
private:
    VertexList           allVertices;
    EdgeList             allEdges;
    std::vector<CdtTri*> allTriangles;
    std::stack<Edge*>    pendingDelaunayStack;  // loose edges awaiting legalization
    std::stack<Edge*>    horzEdgeStack;         // horizontal edges deferred per Y-band
    std::stack<Vertex2*> locMinStack;           // inner local minima (holes)
    bool                 useDelaunay = true;
    Vertex2*             lowermostVertex = nullptr;
    Edge*                firstActive = nullptr; // head of doubly-linked active-edge list
    int64_t              sweepY_ = 0;

    // Comparator: orders active edges by x-intercept at current sweepY_.
    // Non-crossing invariant (CDT input) ensures relative order is stable across y-bands.
    struct EdgeXCmp {
        const int64_t* pY = nullptr;
        static double xAt(const Edge* e, int64_t y) noexcept {
            int64_t dy = e->vB->pt.y - e->vT->pt.y;
            if (dy == 0) return (double)(e->vT->pt.x + e->vB->pt.x) * 0.5;
            return e->vT->pt.x + (double)(e->vB->pt.x - e->vT->pt.x)
                   * (double)(y - e->vT->pt.y) / (double)dy;
        }
        bool operator()(const Edge* a, const Edge* b) const noexcept {
            double xa = xAt(a, *pY), xb = xAt(b, *pY);
            if (xa != xb) return xa < xb;
            return a < b; // pointer tiebreak for strict weak ordering
        }
    };
    std::set<Edge*, EdgeXCmp> activeSet_;

    void      AddPath(const Path64& path);
    bool      AddPaths(const Paths64& paths);
    void      CleanUp();
    void      MergeDupOrCollinearVertices();
    void      SplitEdge(Edge* longE, Edge* shortE);
    Edge*     CreateEdge(Vertex2* v1, Vertex2* v2, EdgeKind k);
    CdtTri*   CreateTriangle(Edge* e1, Edge* e2, Edge* e3);
    Edge*     CreateInnerLocMinLooseEdge(Vertex2* vAbove);
    Edge*     HorizontalBetween(Vertex2* v1, Vertex2* v2);
    void      DoTriangulateLeft(Edge* edge, Vertex2* pivot, int64_t minY);
    void      DoTriangulateRight(Edge* edge, Vertex2* pivot, int64_t minY);
    void      AddEdgeToActives(Edge* edge);
    void      RemoveEdgeFromActives(Edge* edge);
    void      ForceLegal(Edge* edge);

public:
    explicit Delaunay(bool delaunay = true)
        : useDelaunay(delaunay), activeSet_(EdgeXCmp{&sweepY_}) {}
    ~Delaunay() { CleanUp(); }
    Paths64 Execute(const Paths64& paths, TriangulateResult& triResult);
};

void Delaunay::CleanUp()
{
    for (auto v : allVertices) delete v;
    allVertices.resize(0);
    for (auto e : allEdges) delete e;
    allEdges.resize(0);
    for (auto t : allTriangles) delete t;
    allTriangles.resize(0);
    activeSet_.clear();
    firstActive = nullptr;
    lowermostVertex = nullptr;
}

// Prepend edge to the active-edge doubly-linked list and insert into BST.
void Delaunay::AddEdgeToActives(Edge* edge)
{
    if (edge->isActive) return;
    edge->prevE = nullptr;
    edge->nextE = firstActive;
    edge->isActive = true;
    if (firstActive) firstActive->prevE = edge;
    firstActive = edge;
    activeSet_.insert(edge);
}

// Remove edge from active list/BST and from both endpoint vertex edge-lists.
void Delaunay::RemoveEdgeFromActives(Edge* edge)
{
    activeSet_.erase(edge);
    RemoveEdgeFromVertex(edge->vB, edge);
    RemoveEdgeFromVertex(edge->vT, edge);
    Edge* prev = edge->prevE;
    Edge* next = edge->nextE;
    if (next) next->prevE = prev;
    if (prev) prev->nextE = next;
    edge->isActive = false;
    if (firstActive == edge) firstActive = next;
}

// Allocate and wire up a new edge between v1 and v2.
// Loose edges are immediately added to the active list and Delaunay stack.
Edge* Delaunay::CreateEdge(Vertex2* v1, Vertex2* v2, EdgeKind k)
{
    Edge* res = allEdges.emplace_back(new Edge());
    if (v1->pt.y == v2->pt.y)     { res->vB = v1; res->vT = v2; }
    else if (v1->pt.y < v2->pt.y) { res->vB = v2; res->vT = v1; }
    else                           { res->vB = v1; res->vT = v2; }

    if (v1->pt.x <= v2->pt.x) { res->vL = v1; res->vR = v2; }
    else                       { res->vL = v2; res->vR = v1; }
    res->kind = k;
    v1->edges.push_back(res);
    v2->edges.push_back(res);

    if (k == EdgeKind::loose) {
        pendingDelaunayStack.push(res);
        AddEdgeToActives(res);
    }
    return res;
}

// Allocate a triangle and register it with each of its three edges.
// When an edge gets its second triangle it is removed from the active list.
CdtTri* Delaunay::CreateTriangle(Edge* e1, Edge* e2, Edge* e3)
{
    CdtTri* res = allTriangles.emplace_back(new CdtTri(e1, e2, e3));
    for (int i = 0; i < 3; ++i) {
        if (res->edges[i]->triA) {
            res->edges[i]->triB = res;
            RemoveEdgeFromActives(res->edges[i]);
        } else {
            res->edges[i]->triA = res;
            if (!IsLooseEdge(*res->edges[i]))
                RemoveEdgeFromActives(res->edges[i]);
        }
    }
    return res;
}

// Shorten longE so it ends at shortE->vT, then create a new edge from there to the old top.
void Delaunay::SplitEdge(Edge* longE, Edge* shortE)
{
    auto oldT = longE->vT, newT = shortE->vT;
    RemoveEdgeFromVertex(oldT, longE);
    longE->vT = newT;
    if (longE->vL == oldT) longE->vL = newT;
    else                   longE->vR = newT;
    newT->edges.push_back(longE);
    CreateEdge(newT, oldT, longE->kind);
}

// After sorting, merge duplicate vertices and split collinear edges that overlap.
void Delaunay::MergeDupOrCollinearVertices()
{
    auto vIter1 = allVertices.begin();
    for (auto vIter2 = allVertices.begin() + 1; vIter2 != allVertices.end(); ++vIter2) {
        if ((*vIter1)->pt != (*vIter2)->pt) { vIter1 = vIter2; continue; }

        Vertex2* v1 = *vIter1, *v2 = *vIter2;
        if (!v1->innerLM || !v2->innerLM) v1->innerLM = false;

        for (auto e : v2->edges) {
            if (e->vB == v2) e->vB = v1; else e->vT = v1;
            if (e->vL == v2) e->vL = v1; else e->vR = v1;
        }
        std::copy(v2->edges.begin(), v2->edges.end(), back_inserter(v1->edges));
        v2->edges.resize(0);

        auto edges_snapshot = v1->edges;
        for (auto itE = edges_snapshot.begin(); itE != edges_snapshot.end(); ++itE) {
            if (IsHorizontal(*(*itE)) || (*itE)->vB != v1) continue;
            for (auto itE2 = edges_snapshot.begin(); itE2 != edges_snapshot.end(); ++itE2) {
                if (itE2 == itE) continue;
                auto e1 = *itE, e2 = *itE2;
                if (e2->vB != v1 || e1->vT->pt.y == e2->vT->pt.y ||
                    (CrossProductSign(e1->vT->pt, v1->pt, e2->vT->pt) != 0)) continue;
                if (e1->vT->pt.y < e2->vT->pt.y) SplitEdge(e1, e2);
                else                              SplitEdge(e2, e1);
                break;
            }
        }
    }
}

// For a hole local minimum vAbove, find the nearest active edge below it and
// connect vAbove to a visible vertex on that edge with a new loose edge.
Edge* Delaunay::CreateInnerLocMinLooseEdge(Vertex2* vAbove)
{
    if (!firstActive) return nullptr;

    int64_t xAbove = vAbove->pt.x;
    int64_t yAbove = vAbove->pt.y;
    sweepY_ = yAbove;

    Edge* eBelow = nullptr;
    double bestD = -1.0;
    Edge* e = firstActive;
    while (e) {
        if (e->vL->pt.x <= xAbove && e->vR->pt.x >= xAbove &&
            e->vB->pt.y >= yAbove && e->vB != vAbove && e->vT != vAbove &&
            !LeftTurning(e->vL->pt, vAbove->pt, e->vR->pt)) {
            double d = ShortestDistFromSegment(vAbove->pt, e->vL->pt, e->vR->pt);
            if (!eBelow || d < bestD) { eBelow = e; bestD = d; }
        }
        e = e->nextE;
    }

    if (!eBelow) return nullptr;

    Vertex2* vBest = (eBelow->vT->pt.y <= yAbove) ? eBelow->vB : eBelow->vT;
    int64_t xBest = vBest->pt.x;
    int64_t yBest = vBest->pt.y;

    // Refine: if any active edge crosses the tentative connection, prefer its endpoint.
    e = firstActive;
    if (xBest < xAbove) {
        while (e) {
            if (e->vR->pt.x > xBest && e->vL->pt.x < xAbove &&
                e->vB->pt.y > yAbove && e->vT->pt.y < yBest &&
                (SegsIntersect(e->vB->pt, e->vT->pt,
                    vBest->pt, vAbove->pt) == IntersectKind::intersect))
            {
                vBest = (e->vT->pt.y > yAbove) ? e->vT : e->vB;
                xBest = vBest->pt.x;
                yBest = vBest->pt.y;
            }
            e = e->nextE;
        }
    } else {
        while (e) {
            if (e->vR->pt.x < xBest && e->vL->pt.x > xAbove &&
                e->vB->pt.y > yAbove && e->vT->pt.y < yBest &&
                (SegsIntersect(e->vB->pt, e->vT->pt,
                    vBest->pt, vAbove->pt) == IntersectKind::intersect))
            {
                vBest = e->vT->pt.y > yAbove ? e->vT : e->vB;
                xBest = vBest->pt.x;
                yBest = vBest->pt.y;
            }
            e = e->nextE;
        }
    }
    return CreateEdge(vBest, vAbove, EdgeKind::loose);
}

// Find an active horizontal edge whose span lies strictly between v1 and v2.
Edge* Delaunay::HorizontalBetween(Vertex2* v1, Vertex2* v2)
{
    int64_t y = v1->pt.y, l, r;
    if (v1->pt.x > v2->pt.x) { l = v2->pt.x; r = v1->pt.x; }
    else                      { l = v1->pt.x; r = v2->pt.x; }

    Edge* res = firstActive;
    while (res) {
        if (res->vL->pt.y == y && res->vR->pt.y == y &&
            res->vL->pt.x >= l && res->vR->pt.x <= r &&
            (res->vL->pt.x != l || res->vL->pt.x != r)) break;
        res = res->nextE;
    }
    return res;
}

// Recursively fan-fill triangles to the left of `edge` around `pivot`,
// stopping when we drop below minY.
void Delaunay::DoTriangulateLeft(Edge* edge, Vertex2* pivot, int64_t minY)
{
    Vertex2* vAlt = nullptr;
    Edge*    eAlt = nullptr;
    Vertex2* v = (edge->vB == pivot) ? edge->vT : edge->vB;

    for (auto e : pivot->edges) {
        if (e == edge || !e->isActive) continue;
        Vertex2* vX = e->vT == pivot ? e->vB : e->vT;
        if (vX == v) continue;

        int cps = CrossProductSign(v->pt, pivot->pt, vX->pt);
        if (cps == 0) {
            if ((v->pt.x > pivot->pt.x) == (pivot->pt.x > vX->pt.x)) continue;
        } else if (cps > 0 || (vAlt && !LeftTurning(vX->pt, pivot->pt, vAlt->pt)))
            continue;
        vAlt = vX; eAlt = e;
    }

    if (!vAlt || vAlt->pt.y < minY) return;

    if (vAlt->pt.y < pivot->pt.y) { if (IsLeftEdge(*eAlt)) return; }
    else if (vAlt->pt.y > pivot->pt.y) { if (IsRightEdge(*eAlt)) return; }

    Edge* eX = FindLinkingEdge(vAlt, v, (vAlt->pt.y < v->pt.y));
    if (!eX) {
        if (vAlt->pt.y == v->pt.y && v->pt.y == minY && HorizontalBetween(vAlt, v)) return;
        eX = CreateEdge(vAlt, v, EdgeKind::loose);
    }

    CreateTriangle(edge, eAlt, eX);
    if (!EdgeCompleted(eX)) DoTriangulateLeft(eX, vAlt, minY);
}

// Mirror of DoTriangulateLeft for right-turning fans.
void Delaunay::DoTriangulateRight(Edge* edge, Vertex2* pivot, int64_t minY)
{
    Vertex2* vAlt = nullptr;
    Edge*    eAlt = nullptr;
    Vertex2* v = (edge->vB == pivot) ? edge->vT : edge->vB;

    for (auto e : pivot->edges) {
        if (e == edge || !e->isActive) continue;
        Vertex2* vX = e->vT == pivot ? e->vB : e->vT;
        if (vX == v) continue;

        int cps = CrossProductSign(v->pt, pivot->pt, vX->pt);
        if (cps == 0) {
            if ((v->pt.x > pivot->pt.x) == (pivot->pt.x > vX->pt.x)) continue;
        } else if (cps < 0 || (vAlt && !RightTurning(vX->pt, pivot->pt, vAlt->pt)))
            continue;
        vAlt = vX; eAlt = e;
    }

    if (!vAlt || vAlt->pt.y < minY) return;

    if (vAlt->pt.y < pivot->pt.y) { if (IsRightEdge(*eAlt)) return; }
    else if (vAlt->pt.y > pivot->pt.y) { if (IsLeftEdge(*eAlt)) return; }

    Edge* eX = FindLinkingEdge(vAlt, v, (vAlt->pt.y > v->pt.y));
    if (!eX) {
        if (vAlt->pt.y == v->pt.y && v->pt.y == minY && HorizontalBetween(vAlt, v)) return;
        eX = CreateEdge(vAlt, v, EdgeKind::loose);
    }

    CreateTriangle(edge, eX, eAlt);
    if (!EdgeCompleted(eX)) DoTriangulateRight(eX, vAlt, minY);
}

// Delaunay legalization: if the quad formed by `edge` and its two triangles
// violates the in-circle condition, flip the diagonal and recurse on neighbors.
void Delaunay::ForceLegal(Edge* edge)
{
    if (!edge->triA || !edge->triB) return;

    Vertex2* vertA = nullptr;
    Vertex2* vertB = nullptr;
    Edge* edgesA[3] = { nullptr, nullptr, nullptr };
    Edge* edgesB[3] = { nullptr, nullptr, nullptr };

    for (int i = 0; i < 3; ++i) {
        if (edge->triA->edges[i] == edge) continue;
        switch (EdgeContains(edge->triA->edges[i], edge->vL)) {
        case EdgeContainsResult::left:
            edgesA[1] = edge->triA->edges[i];
            vertA = edge->triA->edges[i]->vR;
            break;
        case EdgeContainsResult::right:
            edgesA[1] = edge->triA->edges[i];
            vertA = edge->triA->edges[i]->vL;
            break;
        default:
            edgesB[1] = edge->triA->edges[i];
        }
    }

    for (int i = 0; i < 3; ++i) {
        if (edge->triB->edges[i] == edge) continue;
        switch (EdgeContains(edge->triB->edges[i], edge->vL)) {
        case EdgeContainsResult::left:
            edgesA[2] = edge->triB->edges[i];
            vertB = edge->triB->edges[i]->vR;
            break;
        case EdgeContainsResult::right:
            edgesA[2] = edge->triB->edges[i];
            vertB = edge->triB->edges[i]->vL;
            break;
        default:
            edgesB[2] = edge->triB->edges[i];
        }
    }

    if (CrossProductSign(vertA->pt, edge->vL->pt, edge->vR->pt) == 0) return;

    double ict = InCircleTest(vertA->pt, edge->vL->pt, edge->vR->pt, vertB->pt);
    if (ict == 0 ||
        (RightTurning(vertA->pt, edge->vL->pt, edge->vR->pt) == (ict < 0))) return;

    // Flip: redirect the shared edge to connect vertA ↔ vertB.
    edge->vL = vertA;
    edge->vR = vertB;

    edge->triA->edges[0] = edge;
    for (int i = 1; i < 3; ++i) {
        edge->triA->edges[i] = edgesA[i];
        if (!edgesA[i]) DoError(0);
        if (IsLooseEdge(*edgesA[i])) pendingDelaunayStack.push(edgesA[i]);
        if (edgesA[i]->triA == edge->triA || edgesA[i]->triB == edge->triA) continue;
        if (edgesA[i]->triA == edge->triB)       edgesA[i]->triA = edge->triA;
        else if (edgesA[i]->triB == edge->triB)  edgesA[i]->triB = edge->triA;
        else DoError(0);
    }

    edge->triB->edges[0] = edge;
    for (int i = 1; i < 3; ++i) {
        edge->triB->edges[i] = edgesB[i];
        if (!edgesB[i]) DoError(0);
        if (IsLooseEdge(*edgesB[i])) pendingDelaunayStack.push(edgesB[i]);
        if (edgesB[i]->triA == edge->triB || edgesB[i]->triB == edge->triB) continue;
        if (edgesB[i]->triA == edge->triA)       edgesB[i]->triA = edge->triB;
        else if (edgesB[i]->triB == edge->triA)  edgesB[i]->triB = edge->triB;
        else DoError(0);
    }
}

// Walk one closed path and register its vertices and boundary edges.
// Classifies each run as ascending (left boundary) or descending (right boundary).
void Delaunay::AddPath(const Path64& path)
{
    size_t len = path.size(), i0 = 0, iPrev, iNext;
    if (!FindLocMinIdx(path, len, i0)) return;
    iPrev = Prev(i0, len);
    while (path[iPrev] == path[i0]) iPrev = Prev(iPrev, len);
    iNext = Next(i0, len);

    size_t i = i0;
    while (CrossProductSign(path[iPrev], path[i], path[iNext]) == 0) {
        FindLocMinIdx(path, len, i);
        if (i == i0) return;
        iPrev = Prev(i, len);
        while (path[iPrev] == path[i]) iPrev = Prev(iPrev, len);
        iNext = Next(i, len);
    }

    size_t vert_cnt = allVertices.size();
    Vertex2* v0 = allVertices.emplace_back(new Vertex2(path[i]));
    if (LeftTurning(path[iPrev], path[i], path[iNext])) v0->innerLM = true;
    Vertex2* vPrev = v0;
    i = iNext;

    for (;;) {
        locMinStack.push(vPrev);
        if (!lowermostVertex ||
            vPrev->pt.y > lowermostVertex->pt.y ||
            (vPrev->pt.y == lowermostVertex->pt.y && vPrev->pt.x < lowermostVertex->pt.x))
            lowermostVertex = vPrev;

        iNext = Next(i, len);
        if (CrossProductSign(vPrev->pt, path[i], path[iNext]) == 0) {
            i = iNext; continue;
        }

        while (path[i].y <= vPrev->pt.y) {
            Vertex2* v = allVertices.emplace_back(new Vertex2(path[i]));
            CreateEdge(vPrev, v, EdgeKind::ascend);
            vPrev = v;
            i = iNext;
            iNext = Next(i, len);
            while (CrossProductSign(vPrev->pt, path[i], path[iNext]) == 0) {
                i = iNext; iNext = Next(i, len);
            }
        }

        Vertex2* vPrevPrev = vPrev;
        while (i != i0 && path[i].y >= vPrev->pt.y) {
            Vertex2* v = allVertices.emplace_back(new Vertex2(path[i]));
            CreateEdge(v, vPrev, EdgeKind::descend);
            vPrevPrev = vPrev;
            vPrev = v;
            i = iNext;
            iNext = Next(i, len);
            while (CrossProductSign(vPrev->pt, path[i], path[iNext]) == 0) {
                i = iNext; iNext = Next(i, len);
            }
        }

        if (i == i0) break;
        if (LeftTurning(vPrevPrev->pt, vPrev->pt, path[i])) vPrev->innerLM = true;
    }
    CreateEdge(v0, vPrev, EdgeKind::descend);

    // Discard degenerate triangles (< 3 distinct vertices or nearly coincident points).
    len = allVertices.size() - vert_cnt;
    i = vert_cnt;
    if (len < 3 || (len == 3 &&
        ((DistSqr(allVertices[i]->pt, allVertices[i+1]->pt) <= 1) ||
         (DistSqr(allVertices[i+1]->pt, allVertices[i+2]->pt) <= 1) ||
         (DistSqr(allVertices[i+2]->pt, allVertices[i]->pt) <= 1))))
    {
        for (size_t j = vert_cnt; j < allVertices.size(); ++j)
            allVertices[j]->edges.clear();
    }
}

bool Delaunay::AddPaths(const Paths64& paths)
{
    const auto total_vertex_count =
        std::accumulate(paths.begin(), paths.end(), size_t(0),
            [](const auto& a, const Path64& path) { return a + path.size(); });
    if (total_vertex_count == 0) return false;
    allVertices.reserve(allVertices.capacity() + total_vertex_count);
    allEdges.reserve(allEdges.capacity() + total_vertex_count);
    for (const Path64& path : paths) AddPath(path);
    return allVertices.size() > 2;
}

// Main entry: sort vertices top-to-bottom, sweep, then legalize.
Paths64 Delaunay::Execute(const Paths64& paths, TriangulateResult& triResult)
{
    if (!AddPaths(paths)) {
        triResult = TriangulateResult::no_polygons;
        return Paths64();
    }

    // If the lowest vertex is a hole local minimum the outer path was wound CW;
    // flip all edge kinds so the algorithm treats the outer path as the boundary.
    if (lowermostVertex->innerLM) {
        Vertex2* lm;
        while (!locMinStack.empty()) {
            lm = locMinStack.top();
            lm->innerLM = !lm->innerLM;
            locMinStack.pop();
        }
        for (Edge* e : allEdges)
            if (e->kind == EdgeKind::ascend) e->kind = EdgeKind::descend;
            else                             e->kind = EdgeKind::ascend;
    } else {
        while (!locMinStack.empty()) locMinStack.pop();
    }

    std::sort(allEdges.begin(), allEdges.end(), EdgeListSort);
    std::sort(allVertices.begin(), allVertices.end(), VertexListSort);
    MergeDupOrCollinearVertices();

    int64_t currY = allVertices[0]->pt.y;
    sweepY_ = currY;
    for (auto vIt = allVertices.begin(); vIt != allVertices.end(); ++vIt) {
        Vertex2* v = *vIt;
        if (v->edges.empty()) continue;
        if (v->pt.y != currY) {
            // Process inner local minima (holes) accumulated at the previous Y level.
            while (!locMinStack.empty()) {
                Vertex2* lm = locMinStack.top();
                locMinStack.pop();
                Edge* e = CreateInnerLocMinLooseEdge(lm);
                if (!e) {
                    CleanUp();
                    triResult = TriangulateResult::fail;
                    return Paths64();
                }

                if (IsHorizontal(*e)) {
                    if (e->vL == e->vB) DoTriangulateLeft(e, e->vB, currY);
                    else                DoTriangulateRight(e, e->vB, currY);
                } else {
                    DoTriangulateLeft(e, e->vB, currY);
                    if (!EdgeCompleted(e)) DoTriangulateRight(e, e->vB, currY);
                }

                AddEdgeToActives(lm->edges[0]);
                AddEdgeToActives(lm->edges[1]);
            }

            // Flush horizontal edges deferred from the previous Y band.
            while (!horzEdgeStack.empty()) {
                Edge* e = horzEdgeStack.top();
                horzEdgeStack.pop();
                if (EdgeCompleted(e)) continue;
                if (e->vB == e->vL) {
                    if (IsLeftEdge(*e)) DoTriangulateLeft(e, e->vB, currY);
                } else {
                    if (IsRightEdge(*e)) DoTriangulateRight(e, e->vB, currY);
                }
            }
            currY = v->pt.y;
            sweepY_ = currY;
        }

        for (int i = static_cast<int>(v->edges.size()) - 1; i >= 0; --i) {
            if (i >= static_cast<int>(v->edges.size())) continue;
            Edge* e = v->edges[i];
            if (EdgeCompleted(e) || IsLooseEdge(*e)) continue;

            if (v == e->vB) {
                if (IsHorizontal(*e)) horzEdgeStack.push(e);
                if (!v->innerLM) AddEdgeToActives(e);
            } else {
                if (IsHorizontal(*e)) horzEdgeStack.push(e);
                else if (IsLeftEdge(*e))  DoTriangulateLeft(e, e->vB, v->pt.y);
                else                      DoTriangulateRight(e, e->vB, v->pt.y);
            }
        }

        if (v->innerLM) locMinStack.push(v);
    }

    while (!horzEdgeStack.empty()) {
        Edge* e = horzEdgeStack.top();
        horzEdgeStack.pop();
        if (!EdgeCompleted(e) && e->vB == e->vL)
            DoTriangulateLeft(e, e->vB, currY);
    }

    // Legalize all interior diagonal edges.
    if (useDelaunay) {
        while (!pendingDelaunayStack.empty()) {
            Edge* e = pendingDelaunayStack.top();
            pendingDelaunayStack.pop();
            ForceLegal(e);
        }
    }

    // Collect CCW triangles.
    Paths64 res;
    res.reserve(allTriangles.size());
    for (auto tri : allTriangles) {
        Path64 p = PathFromTriangle(*tri);
        int cps = CrossProductSign(p[0], p[1], p[2]);
        if (cps == 0) continue;
        if (cps < 0) std::reverse(p.begin(), p.end());
        res.push_back(p);
    }

    CleanUp();
    triResult = TriangulateResult::success;
    return res;
}

} // anonymous namespace

// ---- Public wrapper ----

// Triangulate a polygon with optional holes.
// Coordinates are scaled to integer space for exact arithmetic, then mapped back
// to indices into the flat vertex array [border..., hole0..., hole1...].
std::vector<std::array<int,3>> cdt_triangulate(
    const std::vector<std::pair<double,double>>& border_2d,
    const std::vector<std::vector<std::pair<double,double>>>& holes_2d)
{
    const double scale = 1e6;
    auto to_pt64 = [&](double x, double y) -> Point64 {
        return { int64_t(std::round(x * scale)), int64_t(std::round(y * scale)) };
    };

    std::vector<std::pair<double,double>> flat;
    for (auto& p : border_2d) flat.push_back(p);
    for (auto& h : holes_2d) for (auto& p : h) flat.push_back(p);

    std::unordered_map<std::pair<int64_t,int64_t>, int, P64Hash> pt_map;
    pt_map.reserve(flat.size());
    for (int i = 0; i < (int)flat.size(); ++i)
        pt_map.emplace(std::make_pair(
            int64_t(std::round(flat[i].first  * scale)),
            int64_t(std::round(flat[i].second * scale))), i);

    auto make_path = [&](const std::vector<std::pair<double,double>>& pts) {
        Path64 p;
        for (auto& v : pts) p.push_back(to_pt64(v.first, v.second));
        if (p.size() > 1 && p.front() == p.back()) p.pop_back();
        return p;
    };

    Paths64 input;
    input.push_back(make_path(border_2d));
    for (auto& h : holes_2d) input.push_back(make_path(h));

    TriangulateResult result;
    Delaunay d(true);
    Paths64 tris = d.Execute(input, result);

    std::vector<std::array<int,3>> out;
    for (auto& tri : tris) {
        if (tri.size() != 3) continue;
        std::array<int,3> f;
        bool ok = true;
        for (int k = 0; k < 3; ++k) {
            auto it = pt_map.find({tri[k].x, tri[k].y});
            if (it == pt_map.end()) { ok = false; break; }
            f[k] = it->second;
        }
        if (ok) out.push_back(f);
    }
    return out;
}

bool point_in_polygon_2d(double px, double py, const std::vector<double>& coords) {
    auto cross_2d = [](double ax, double ay, double bx, double by, double cx, double cy) -> double {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    };
    int winding = 0;
    size_t n = coords.size() / 2;
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        double y0 = coords[i * 2 + 1], y1 = coords[j * 2 + 1];
        if (y0 <= py) {
            if (y1 > py) {
                double x0 = coords[i * 2], x1 = coords[j * 2];
                if (cross_2d(x0, y0, x1, y1, px, py) > 0.0) ++winding;
            }
        } else {
            if (y1 <= py) {
                double x0 = coords[i * 2], x1 = coords[j * 2];
                if (cross_2d(x0, y0, x1, y1, px, py) < 0.0) --winding;
            }
        }
    }
    return winding != 0;
}

namespace {

static std::vector<Point> strip_close_pl(const Polyline& pl) {
    auto pts = pl.get_points();
    if (pts.size() > 1) {
        const auto& f = pts.front();
        const auto& b = pts.back();
        if (std::abs(f[0]-b[0]) < 1e-12 && std::abs(f[1]-b[1]) < 1e-12 && std::abs(f[2]-b[2]) < 1e-12)
            pts.pop_back();
    }
    return pts;
}

static double signed_area_2d(const std::vector<std::pair<double,double>>& pts) {
    double area = 0.0;
    size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += pts[i].first * pts[j].second - pts[j].first * pts[i].second;
    }
    return area * 0.5;
}

} // anonymous namespace

std::vector<std::array<int,3>> RemeshCDT::triangulate(const std::vector<Polyline>& polylines) {
    if (polylines.empty()) return {};
    auto border_pts = strip_close_pl(polylines[0]);
    if (border_pts.size() < 3) return {};
    std::vector<std::pair<double,double>> boundary_2d;
    for (const auto& p : border_pts) boundary_2d.push_back({p[0], p[1]});
    std::vector<std::vector<std::pair<double,double>>> holes_2d;
    for (size_t i = 1; i < polylines.size(); ++i) {
        auto h = strip_close_pl(polylines[i]);
        if (h.size() < 3) continue;
        std::vector<std::pair<double,double>> h2d;
        for (const auto& p : h) h2d.push_back({p[0], p[1]});
        holes_2d.push_back(std::move(h2d));
    }
    return cdt_triangulate(boundary_2d, holes_2d);
}


Mesh RemeshCDT::from_polylines(const std::vector<Polyline>& polylines, bool is_2d, bool is_first_boundary) {
    if (polylines.empty()) return Mesh();

    // Find border index
    int border_idx = 0;
    if (!is_first_boundary && polylines.size() > 1) {
        double max_diag = 0.0;
        for (size_t i = 0; i < polylines.size(); ++i) {
            auto pts = polylines[i].get_points();
            if (pts.size() < 3) continue;
            double minx = pts[0][0], miny = pts[0][1], minz = pts[0][2];
            double maxx = minx, maxy = miny, maxz = minz;
            for (const auto& p : pts) {
                if (p[0] < minx) minx = p[0]; if (p[0] > maxx) maxx = p[0];
                if (p[1] < miny) miny = p[1]; if (p[1] > maxy) maxy = p[1];
                if (p[2] < minz) minz = p[2]; if (p[2] > maxz) maxz = p[2];
            }
            double dx = maxx-minx, dy = maxy-miny, dz = maxz-minz;
            double diag = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (diag > max_diag) { max_diag = diag; border_idx = static_cast<int>(i); }
        }
    }

    // Strip closing duplicates
    auto border = strip_close_pl(polylines[border_idx]);
    if (border.size() < 3) return Mesh();

    std::vector<std::vector<Point>> hole_pts_3d;
    for (size_t i = 0; i < polylines.size(); ++i) {
        if (static_cast<int>(i) == border_idx) continue;
        auto h = strip_close_pl(polylines[i]);
        if (h.size() < 3) continue;
        hole_pts_3d.push_back(h);
    }

    // Project to 2D
    std::vector<std::pair<double,double>> boundary_2d;
    std::vector<std::vector<std::pair<double,double>>> holes_2d;

    if (is_2d) {
        for (const auto& p : border) boundary_2d.push_back({p[0], p[1]});
        for (auto& hole : hole_pts_3d) {
            std::vector<std::pair<double,double>> h2d;
            for (const auto& p : hole) h2d.push_back({p[0], p[1]});
            holes_2d.push_back(std::move(h2d));
        }
    } else {
        std::vector<Point> all_pts = border;
        for (const auto& h : hole_pts_3d)
            for (const auto& p : h) all_pts.push_back(p);
        Polyline all_pl(all_pts);
        Point origin;
        Vector xaxis, yaxis, zaxis;
        all_pl.get_average_plane(origin, xaxis, yaxis, zaxis);

        auto project_2d = [&](const Point& p) -> std::pair<double,double> {
            double dx = p[0]-origin[0], dy = p[1]-origin[1], dz = p[2]-origin[2];
            return { dx*xaxis[0]+dy*xaxis[1]+dz*xaxis[2],
                     dx*yaxis[0]+dy*yaxis[1]+dz*yaxis[2] };
        };
        for (const auto& p : border) boundary_2d.push_back(project_2d(p));
        for (auto& hole : hole_pts_3d) {
            std::vector<std::pair<double,double>> h2d;
            for (const auto& p : hole) h2d.push_back(project_2d(p));
            holes_2d.push_back(std::move(h2d));
        }
    }

    // Enforce CCW border, CW holes
    if (signed_area_2d(boundary_2d) < 0.0) {
        std::reverse(border.begin(), border.end());
        std::reverse(boundary_2d.begin(), boundary_2d.end());
    }
    for (size_t i = 0; i < hole_pts_3d.size(); ++i) {
        if (signed_area_2d(holes_2d[i]) > 0.0) {
            std::reverse(hole_pts_3d[i].begin(), hole_pts_3d[i].end());
            std::reverse(holes_2d[i].begin(), holes_2d[i].end());
        }
    }

    auto tris = cdt_triangulate(boundary_2d, holes_2d);

    std::vector<Point> all_pts = border;
    for (const auto& h : hole_pts_3d)
        for (const auto& p : h) all_pts.push_back(p);

    Mesh mesh;
    std::vector<size_t> vkeys;
    for (const auto& p : all_pts) vkeys.push_back(mesh.add_vertex(p));

    if (SESSION_CONFIG.explode_mesh_faces) {
        for (const auto& t : tris)
            mesh.add_face({vkeys[t[0]], vkeys[t[1]], vkeys[t[2]]});
        return mesh;
    }

    if (hole_pts_3d.empty()) {
        std::vector<size_t> fvkeys(vkeys.begin(), vkeys.begin() + border.size());
        auto fkey = mesh.add_face(fvkeys);
        if (fkey.has_value()) {
            std::vector<std::array<size_t,3>> tri_list;
            for (const auto& f : tris) {
                if (vkeys[f[0]]==vkeys[f[1]] || vkeys[f[1]]==vkeys[f[2]] || vkeys[f[2]]==vkeys[f[0]]) continue;
                tri_list.push_back({vkeys[f[0]], vkeys[f[1]], vkeys[f[2]]});
            }
            std::unordered_set<size_t> covered;
            for (const auto& t : tri_list) { covered.insert(t[0]); covered.insert(t[1]); covered.insert(t[2]); }
            size_t n_vk = border.size();
            for (size_t m = 0; m < n_vk; ++m) {
                if (!covered.count(vkeys[m]))
                    tri_list.push_back({vkeys[(m+n_vk-1)%n_vk], vkeys[m], vkeys[(m+1)%n_vk]});
            }
            mesh.set_face_triangulation(fkey.value(), std::move(tri_list));
        }
    } else {
        std::vector<size_t> fvkeys(vkeys.begin(), vkeys.begin() + border.size());
        auto fkey = mesh.add_face(fvkeys);
        if (fkey.has_value()) {
            std::vector<std::vector<size_t>> hole_rings;
            size_t off = border.size();
            for (const auto& h : hole_pts_3d) {
                std::vector<size_t> ring(vkeys.begin()+off, vkeys.begin()+off+h.size());
                hole_rings.push_back(ring); off += h.size();
            }
            mesh.set_face_holes(fkey.value(), hole_rings);
            std::vector<std::array<size_t,3>> tri_list;
            for (const auto& f : tris)
                if (vkeys[f[0]]!=vkeys[f[1]] && vkeys[f[1]]!=vkeys[f[2]] && vkeys[f[2]]!=vkeys[f[0]])
                    tri_list.push_back({vkeys[f[0]], vkeys[f[1]], vkeys[f[2]]});
            mesh.set_face_triangulation(fkey.value(), std::move(tri_list));
        }
    }
    return mesh;
}

} // namespace session_cpp
