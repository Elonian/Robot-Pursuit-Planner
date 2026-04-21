#ifndef STP_SPACE_TIME_ASTAR_H
#define STP_SPACE_TIME_ASTAR_H

#include "../../utils/grid_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace stp {

struct QueueItem {
    double f = 0.0;
    long long g = 0;
    int node_id = -1;

    bool operator<(const QueueItem& other) const {
        if (f == other.f) {
            return g < other.g;
        }
        return f > other.f;
    }
};

inline double movingTargetHeuristic(const PlanningRequest& req, int cell, int t) {
    Pose p = toPose(cell, req.x_size);
    int remaining = req.target_steps - 1 - t;
    if (remaining <= 0) {
        return 0.0;
    }

    int window = std::min(remaining, 640);
    int stride = 1;
    if (window > 384) {
        stride = 8;
    } else if (window > 160) {
        stride = 4;
    } else if (window > 64) {
        stride = 2;
    }

    double best = std::numeric_limits<double>::infinity();
    for (int dt = 0; dt <= window; dt += stride) {
        int tt = t + dt;
        Pose target = targetAt(req, tt);
        int dist = chebyshev(p.x, p.y, target.x, target.y);
        int infeasible = std::max(0, dist - dt);
        double score = static_cast<double>(req.min_free_cost) *
                       static_cast<double>(dist + 4 * infeasible);
        best = std::min(best, score);
    }

    Pose final_target = targetAt(req, req.target_steps - 1);
    int final_dist = chebyshev(p.x, p.y, final_target.x, final_target.y);
    best = std::min(best, static_cast<double>(req.min_free_cost) * final_dist);
    return std::isfinite(best) ? best : 0.0;
}

class SpaceTimeWeightedAStar {
public:
    bool plan(
        const PlanningRequest& req,
        std::chrono::steady_clock::time_point start_time,
        double budget_ms,
        std::vector<Pose>& out_path) const {
        out_path.clear();
        if (req.curr_time >= req.target_steps - 1) {
            return false;
        }

        const int start_cell = toCell(req.robot_x, req.robot_y, req.x_size);
        const double heuristic_weight = 2.2;
        const int remaining = req.target_steps - req.curr_time;
        const int max_expansions = std::max(60000, std::min(450000, remaining * 260));

        std::vector<Node> nodes;
        nodes.reserve(120000);
        std::priority_queue<QueueItem> open;
        std::unordered_map<uint64_t, long long> best_g;
        best_g.reserve(180000);

        Node start;
        start.cell = start_cell;
        start.t = req.curr_time;
        start.g = 0;
        nodes.push_back(start);
        best_g[spacetimeKey(start_cell, req.curr_time)] = 0;
        open.push(QueueItem{
            heuristic_weight * movingTargetHeuristic(req, start_cell, req.curr_time),
            0,
            0});

        int expansions = 0;
        while (!open.empty()) {
            if (++expansions > max_expansions || elapsedMs(start_time) > budget_ms) {
                break;
            }

            QueueItem item = open.top();
            open.pop();
            const Node node = nodes[item.node_id];
            auto best_it = best_g.find(spacetimeKey(node.cell, node.t));
            if (best_it == best_g.end() || best_it->second != node.g) {
                continue;
            }
            if (node.t >= req.target_steps - 1) {
                continue;
            }

            Pose p = toPose(node.cell, req.x_size);
            for (int dir = 0; dir < kNumDirs; ++dir) {
                const int nx = p.x + kDx[dir];
                const int ny = p.y + kDy[dir];
                if (!isFree(req, nx, ny)) {
                    continue;
                }

                const int nt = node.t + 1;
                const int ncell = toCell(nx, ny, req.x_size);
                const long long ng = node.g + cellCost(req, node.cell);
                const uint64_t key = spacetimeKey(ncell, nt);
                auto old = best_g.find(key);
                if (old != best_g.end() && old->second <= ng) {
                    continue;
                }
                best_g[key] = ng;

                Node child;
                child.cell = ncell;
                child.t = nt;
                child.g = ng;
                child.parent = item.node_id;
                const int child_id = static_cast<int>(nodes.size());
                nodes.push_back(child);

                if (ncell == targetCellAt(req, nt)) {
                    out_path = reconstruct(nodes, child_id, req.x_size);
                    return out_path.size() >= 2;
                }

                double h = movingTargetHeuristic(req, ncell, nt);
                open.push(QueueItem{static_cast<double>(ng) + heuristic_weight * h, ng, child_id});
            }
        }
        return false;
    }

private:
    struct Node {
        int cell = 0;
        int t = 0;
        long long g = 0;
        int parent = -1;
    };

    static std::vector<Pose> reconstruct(const std::vector<Node>& nodes, int goal_id, int x_size) {
        std::vector<Pose> path;
        int cur = goal_id;
        while (cur >= 0) {
            path.push_back(toPose(nodes[cur].cell, x_size));
            cur = nodes[cur].parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

}  // namespace stp

#endif  // STP_SPACE_TIME_ASTAR_H
