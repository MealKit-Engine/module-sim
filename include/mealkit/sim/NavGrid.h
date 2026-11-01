#pragma once

#include "mealkit/core/Hash.h"
#include "mealkit/core/Reflect.h"
#include "mealkit/physics/PhysicsWorld.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <vector>

// AI pathfinding on a 2D grid with height layers (docs/CODING_MODEL.md "Pathfinding").
//   bakeNavGrid(): after the scene's static bodies are created (before dynamic ones), rays down every cell column find floors
//                  (several per column: bridges, upper floors), keep those with headroom, then shrink by the agent radius.
//   findPath():    A* over the cells (8 neighbors, a step up/down within stepHeight). Integer costs, open list ordered by (cost, node number),
//                  so the result is the same on every peer. Limit the number of searches per tick in the game and run them in a fixed order.
// The grid is derived from scene collision (already in the content hash) on every peer the same way, so it isn't state:
// keep it next to the World like tables, and keep only paths (waypoints) in state. NavSearch is scratch memory, also not state.
namespace mealkit::sim {

MEALKIT_STRUCT(NavBakeSettings) {
    MEALKIT_FIELD(f32, cellSize, 0.5f, "Cell size (m)");
    MEALKIT_FIELD(f32, agentHeight, 1.8f, "Agent height: headroom needed (m)");
    MEALKIT_FIELD(f32, agentRadius, 0.45f, "Agent radius: distance kept from walls and edges (m)");
    MEALKIT_FIELD(f32, stepHeight, 0.4f, "Highest step between neighbor cells (m)");
    MEALKIT_FIELD(u32, maxLayers, 4, "Floors per cell column");
};

struct NavGrid {
    static constexpr u32 kNone = 0xFFFFFFFFu;
    static constexpr u8 kBlocked = 0, kWalkable = 1;

    Vec3 origin; // corner of cell (0, 0) at the bottom of the bake area
    f32 cellSize = 0.5f;
    f32 stepHeight = 0.4f;
    u32 width = 0, depth = 0, layers = 0;
    std::vector<f32> height; // per node: floor height (node = (z * width + x) * layers + layer, layers top to bottom)
    std::vector<u8> flags;   // per node: kWalkable, or kBlocked (also unused layers)

    bool empty() const { return width == 0; }
    u32 nodeCount() const { return width * depth * layers; }
    u32 node(u32 x, u32 z, u32 layer) const { return (z * width + x) * layers + layer; }
    u32 cellX(u32 n) const { return (n / layers) % width; }
    u32 cellZ(u32 n) const { return n / layers / width; }
    bool walkable(u32 n) const { return n < flags.size() && flags[n] == kWalkable; }
    Vec3 position(u32 n) const {
        return {origin.x + (static_cast<f32>(cellX(n)) + 0.5f) * cellSize, height[n],
                origin.z + (static_cast<f32>(cellZ(n)) + 0.5f) * cellSize};
    }
    // Walkable layer of cell (x, z) within stepHeight of y (closest; on a tie the upper one). kNone if none.
    u32 layerNear(u32 x, u32 z, f32 y, f32 reach) const {
        u32 best = kNone;
        f32 bestDy = reach;
        for (u32 l = 0; l < layers; ++l) {
            const u32 n = node(x, z, l);
            if (!walkable(n)) continue;
            const f32 dy = std::fabs(height[n] - y);
            if (dy <= bestDy && (best == kNone || dy < bestDy)) {
                best = n;
                bestDy = dy;
            }
        }
        return best;
    }
    // Nearest walkable node to a feet position: its own cell first, then rings out to `rings` cells (first in x, z order on a tie).
    u32 nearest(Vec3 feet, u32 rings = 2) const {
        if (empty()) return kNone;
        const i32 cx = static_cast<i32>(std::floor((feet.x - origin.x) / cellSize));
        const i32 cz = static_cast<i32>(std::floor((feet.z - origin.z) / cellSize));
        for (i32 r = 0; r <= static_cast<i32>(rings); ++r) {
            u32 best = kNone;
            f32 bestD = 0.0f;
            for (i32 z = cz - r; z <= cz + r; ++z) {
                for (i32 x = cx - r; x <= cx + r; ++x) {
                    if (std::max(std::abs(x - cx), std::abs(z - cz)) != r) continue; // ring only
                    if (x < 0 || z < 0 || x >= static_cast<i32>(width) || z >= static_cast<i32>(depth)) continue;
                    const u32 n = layerNear(static_cast<u32>(x), static_cast<u32>(z), feet.y, stepHeight + 0.5f);
                    if (n == kNone) continue;
                    const Vec3 p = position(n);
                    const f32 d = (p.x - feet.x) * (p.x - feet.x) + (p.z - feet.z) * (p.z - feet.z);
                    if (best == kNone || d < bestD) {
                        best = n;
                        bestD = d;
                    }
                }
            }
            if (best != kNone) return best;
        }
        return kNone;
    }

    void hash(Hasher& hasher) const {
        hasher.add(width);
        hasher.add(depth);
        hasher.add(layers);
        for (u32 n = 0; n < nodeCount(); ++n) {
            hasher.add(flags[n]);
            if (flags[n] == kWalkable) hasher.add(height[n]);
        }
    }
};

// Bakes the area [min, max] (max.y = where rays start) from the physics bodies that exist now.
inline NavGrid bakeNavGrid(const physics::PhysicsWorld& world, Vec3 min, Vec3 max, const NavBakeSettings& s) {
    NavGrid g;
    g.cellSize = std::max(s.cellSize, 0.05f);
    g.stepHeight = s.stepHeight;
    g.origin = min;
    g.width = static_cast<u32>(std::max(0.0f, std::ceil((max.x - min.x) / g.cellSize)));
    g.depth = static_cast<u32>(std::max(0.0f, std::ceil((max.z - min.z) / g.cellSize)));
    g.layers = std::clamp(s.maxLayers, 1u, 16u);
    g.height.assign(g.nodeCount(), 0.0f);
    g.flags.assign(g.nodeCount(), NavGrid::kBlocked);
    const f32 drop = max.y - min.y;
    if (drop <= 0.0f) return g;
    constexpr f32 kSkin = 0.05f; // const-ok: gap above a floor for the headroom ray / below it to look for the next floor

    for (u32 z = 0; z < g.depth; ++z) {
        for (u32 x = 0; x < g.width; ++x) {
            const f32 px = min.x + (static_cast<f32>(x) + 0.5f) * g.cellSize;
            const f32 pz = min.z + (static_cast<f32>(z) + 0.5f) * g.cellSize;
            f32 y = max.y;
            u32 layer = 0;
            // Walk down the column: a hit at fraction 0 means the ray starts inside a solid, so step down and try again
            for (u32 guard = 0; guard < 256 && layer < g.layers && y > min.y; ++guard) {
                physics::RayHit hit;
                if (!world.castRay({px, y, pz}, {0.0f, min.y - y, 0.0f}, hit)) break;
                if (hit.fraction <= 0.0f) {
                    y -= g.cellSize;
                    continue;
                }
                const f32 floor = y + (min.y - y) * hit.fraction;
                physics::RayHit roof;
                const bool clear = !world.castRay({px, floor + kSkin, pz}, {0.0f, s.agentHeight, 0.0f}, roof);
                if (clear) {
                    const u32 n = g.node(x, z, layer++);
                    g.height[n] = floor;
                    g.flags[n] = NavGrid::kWalkable;
                }
                y = floor - kSkin;
            }
        }
    }

    // Keep the agent radius away from walls, drops and the area edge: every cell within r rings needs a floor within stepHeight
    const i32 r = static_cast<i32>(s.agentRadius / g.cellSize + 0.5f);
    if (r > 0) {
        std::vector<u8> kept = g.flags;
        for (u32 z = 0; z < g.depth; ++z) {
            for (u32 x = 0; x < g.width; ++x) {
                for (u32 l = 0; l < g.layers; ++l) {
                    const u32 n = g.node(x, z, l);
                    if (!g.walkable(n)) continue;
                    bool ok = true;
                    for (i32 dz = -r; dz <= r && ok; ++dz) {
                        for (i32 dx = -r; dx <= r && ok; ++dx) {
                            const i32 nx = static_cast<i32>(x) + dx, nz = static_cast<i32>(z) + dz;
                            ok = nx >= 0 && nz >= 0 && nx < static_cast<i32>(g.width) && nz < static_cast<i32>(g.depth) &&
                                 g.layerNear(static_cast<u32>(nx), static_cast<u32>(nz), g.height[n], g.stepHeight) != NavGrid::kNone;
                        }
                    }
                    if (!ok) kept[n] = NavGrid::kBlocked;
                }
            }
        }
        g.flags = std::move(kept);
    }
    return g;
}

enum class PathResult : u8 { None, Found, Partial }; // Partial: search limit hit or goal unreachable → path to the closest node reached

// Scratch memory for findPath (reused; not state, results don't depend on what was in it)
struct NavSearch {
    std::vector<u32> cost, parent, stamp;
    std::vector<u32> nodes; // path nodes, goal first
    u32 run = 0;
};

// A* from a feet position to another. Writes up to maxPoints waypoints (start excluded, straight runs merged) into out.
// maxExpand limits the work per search (nodes taken from the open list).
inline PathResult findPath(const NavGrid& g, NavSearch& s, Vec3 fromFeet, Vec3 toFeet, Vec3* out, u32 maxPoints, u32& count,
                           u32 maxExpand = 4096) {
    count = 0;
    const u32 start = g.nearest(fromFeet), goal = g.nearest(toFeet, 4);
    if (start == NavGrid::kNone) return PathResult::None;
    const u32 total = g.nodeCount();
    if (s.stamp.size() != total) {
        s.cost.assign(total, 0);
        s.parent.assign(total, 0);
        s.stamp.assign(total, 0);
        s.run = 0;
    }
    if (++s.run == 0) { // wrapped around: clear the stamps once
        std::fill(s.stamp.begin(), s.stamp.end(), 0u);
        s.run = 1;
    }
    const u32 run = s.run;
    auto heuristic = [&](u32 n) -> u32 { // octile distance in cells × 10 (goal unreachable → 0, plain Dijkstra toward the limit)
        if (goal == NavGrid::kNone) return 0;
        const u32 dx = static_cast<u32>(std::abs(static_cast<i32>(g.cellX(n)) - static_cast<i32>(g.cellX(goal))));
        const u32 dz = static_cast<u32>(std::abs(static_cast<i32>(g.cellZ(n)) - static_cast<i32>(g.cellZ(goal))));
        return 10 * std::max(dx, dz) + 4 * std::min(dx, dz);
    };
    // Open list key: (estimated total cost << 32) | node number → ties go to the lower node number
    std::priority_queue<u64, std::vector<u64>, std::greater<u64>> open;
    s.stamp[start] = run;
    s.cost[start] = 0;
    s.parent[start] = start;
    open.push(static_cast<u64>(heuristic(start)) << 32 | start);
    u32 best = start, bestH = heuristic(start), expanded = 0;
    bool reached = false;
    static constexpr i32 kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static constexpr i32 kDz[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!open.empty() && expanded < maxExpand) {
        const u64 top = open.top();
        open.pop();
        const u32 n = static_cast<u32>(top & 0xFFFFFFFFu);
        const u32 f = static_cast<u32>(top >> 32);
        if (f != s.cost[n] + heuristic(n)) continue; // stale entry
        ++expanded;
        if (n == goal) {
            best = n;
            reached = true;
            break;
        }
        const u32 h = heuristic(n);
        if (h < bestH) {
            best = n;
            bestH = h;
        }
        const i32 x = static_cast<i32>(g.cellX(n)), z = static_cast<i32>(g.cellZ(n));
        u32 straight[4] = {NavGrid::kNone, NavGrid::kNone, NavGrid::kNone, NavGrid::kNone};
        for (u32 d = 0; d < 8; ++d) {
            const i32 nx = x + kDx[d], nz = z + kDz[d];
            if (nx < 0 || nz < 0 || nx >= static_cast<i32>(g.width) || nz >= static_cast<i32>(g.depth)) continue;
            const u32 m = g.layerNear(static_cast<u32>(nx), static_cast<u32>(nz), g.height[n], g.stepHeight);
            if (d < 4) straight[d] = m;
            if (m == NavGrid::kNone) continue;
            if (d >= 4) { // diagonals only when both sides are open (no corner cutting)
                const u32 sx = kDx[d] > 0 ? 0u : 1u, sz = kDz[d] > 0 ? 2u : 3u;
                if (straight[sx] == NavGrid::kNone || straight[sz] == NavGrid::kNone) continue;
            }
            const u32 c = s.cost[n] + (d < 4 ? 10u : 14u);
            if (s.stamp[m] == run && c >= s.cost[m]) continue;
            s.stamp[m] = run;
            s.cost[m] = c;
            s.parent[m] = n;
            open.push(static_cast<u64>(c + heuristic(m)) << 32 | m);
        }
    }

    s.nodes.clear();
    for (u32 n = best; ; n = s.parent[n]) {
        s.nodes.push_back(n);
        if (n == start) break;
    }
    // Waypoints from the start side, keeping only the nodes where the direction changes (and the last one)
    for (size_t i = s.nodes.size() - 1; i-- > 0 && count < maxPoints;) {
        const u32 n = s.nodes[i];
        if (i > 0) {
            const u32 prev = s.nodes[i + 1], next = s.nodes[i - 1];
            const i32 ax = static_cast<i32>(g.cellX(n)) - static_cast<i32>(g.cellX(prev));
            const i32 az = static_cast<i32>(g.cellZ(n)) - static_cast<i32>(g.cellZ(prev));
            const i32 bx = static_cast<i32>(g.cellX(next)) - static_cast<i32>(g.cellX(n));
            const i32 bz = static_cast<i32>(g.cellZ(next)) - static_cast<i32>(g.cellZ(n));
            if (ax == bx && az == bz && std::fabs(g.height[next] - g.height[n]) < 0.01f) continue;
        }
        out[count++] = g.position(n);
    }
    if (reached) return PathResult::Found;
    return best == start && goal != start ? PathResult::None : PathResult::Partial;
}

} // namespace mealkit::sim
