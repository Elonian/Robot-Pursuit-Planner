#ifndef STP_SPATIAL_INTERCEPT_ASTAR_H
#define STP_SPATIAL_INTERCEPT_ASTAR_H

#include "space_time_astar.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace stp {

class SpatialInterceptAStar {
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
        std::vector<Candidate> candidates = buildCandidates(req);
        if (candidates.empty()) {
            return false;
        }

        std::vector<Pose> best_path;
        int best_intercept_steps = -1;
        long long best_score = kInfCost;
        for (const Candidate& candidate : candidates) {
            if (elapsedMs(start_time) > budget_ms) {
                break;
            }
            std::vector<Pose> path;
            int max_steps = candidate.t - req.curr_time;
            if (!planToCell(req, start_cell, candidate.cell, max_steps, start_time, budget_ms, path)) {
                continue;
            }

            long long travel_cost = scorePathWithEvaluatorCost(req, path);
            long long wait_steps = std::max(0, max_steps - static_cast<int>(path.size()) + 1);
            travel_cost += wait_steps * static_cast<long long>(cellCost(req, candidate.cell));
            if (travel_cost < best_score) {
                best_score = travel_cost;
                best_path = path;
                best_intercept_steps = max_steps;
            }
        }

        if (best_path.size() >= 2) {
            while (best_intercept_steps >= 0 &&
                   static_cast<int>(best_path.size()) < best_intercept_steps + 1) {
                best_path.push_back(best_path.back());
            }
            out_path = best_path;
            return true;
        }
        if (best_path.size() == 1) {
            while (best_intercept_steps >= 0 &&
                   static_cast<int>(best_path.size()) < best_intercept_steps + 1) {
                best_path.push_back(best_path.back());
            }
            if (best_path.size() == 1) {
                best_path.push_back(best_path.back());
            }
            out_path = best_path;
            return true;
        }
        return false;
    }

private:
    struct Candidate {
        int t = 0;
        int cell = 0;
        double score = 0.0;
    };

    struct Node {
        int cell = 0;
        long long g = 0;
        int steps = 0;
        int parent = -1;
    };

    static std::vector<Candidate> buildCandidates(const PlanningRequest& req) {
        std::vector<Candidate> candidates;
        std::unordered_set<int> seen_cells;
        seen_cells.reserve(96);

        for (int t = req.curr_time + 1; t < req.target_steps; ++t) {
            Pose target = targetAt(req, t);
            int dist = chebyshev(req.robot_x, req.robot_y, target.x, target.y);
            int available = t - req.curr_time;
            if (dist > available) {
                continue;
            }
            int cell = toCell(target.x, target.y, req.x_size);
            if (!isFreeCell(req, cell)) {
                continue;
            }
            if (seen_cells.insert(cell).second) {
                double slack = static_cast<double>(available - dist);
                double score = static_cast<double>(available) +
                               0.02 * static_cast<double>(req.map[cell]) * slack +
                               0.5 * static_cast<double>(dist);
                candidates.push_back(Candidate{t, cell, score});
            }
        }

        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
            if (a.score == b.score) {
                return a.t < b.t;
            }
            return a.score < b.score;
        });
        if (candidates.size() > 24) {
            std::vector<Candidate> selected;
            selected.reserve(24);
            for (size_t i = 0; i < std::min<size_t>(12, candidates.size()); ++i) {
                selected.push_back(candidates[i]);
            }
            for (int i = 0; i < 12; ++i) {
                size_t idx = static_cast<size_t>(
                    std::round(static_cast<double>(i) * static_cast<double>(candidates.size() - 1) / 11.0));
                selected.push_back(candidates[idx]);
            }
            std::sort(selected.begin(), selected.end(), [](const Candidate& a, const Candidate& b) {
                return a.t < b.t || (a.t == b.t && a.cell < b.cell);
            });
            selected.erase(
                std::unique(selected.begin(), selected.end(), [](const Candidate& a, const Candidate& b) {
                    return a.t == b.t && a.cell == b.cell;
                }),
                selected.end());
            std::sort(selected.begin(), selected.end(), [](const Candidate& a, const Candidate& b) {
                if (a.score == b.score) {
                    return a.t < b.t;
                }
                return a.score < b.score;
            });
            candidates = selected;
        }
        return candidates;
    }

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

    static bool planToCell(
        const PlanningRequest& req,
        int start_cell,
        int goal_cell,
        int max_steps,
        std::chrono::steady_clock::time_point start_time,
        double budget_ms,
        std::vector<Pose>& out_path) {
        out_path.clear();
        if (max_steps < 0) {
            return false;
        }
        if (start_cell == goal_cell) {
            out_path.push_back(toPose(start_cell, req.x_size));
            return true;
        }

        Pose goal = toPose(goal_cell, req.x_size);
        std::vector<Node> nodes;
        nodes.reserve(60000);
        std::priority_queue<QueueItem> open;
        std::unordered_map<int, long long> best_g;
        best_g.reserve(90000);

        Node start;
        start.cell = start_cell;
        nodes.push_back(start);
        best_g[start_cell] = 0;

        Pose start_pose = toPose(start_cell, req.x_size);
        open.push(QueueItem{
            static_cast<double>(req.min_free_cost) *
                chebyshev(start_pose.x, start_pose.y, goal.x, goal.y),
            0,
            0});

        const int max_expansions = 220000;
        int expansions = 0;
        while (!open.empty()) {
            if (++expansions > max_expansions || elapsedMs(start_time) > budget_ms) {
                break;
            }

            QueueItem item = open.top();
            open.pop();
            const Node node = nodes[item.node_id];
            auto best_it = best_g.find(node.cell);
            if (best_it == best_g.end() || best_it->second != node.g) {
                continue;
            }
            if (node.cell == goal_cell) {
                if (node.steps <= max_steps) {
                    out_path = reconstruct(nodes, item.node_id, req.x_size);
                    return true;
                }
                continue;
            }
            if (node.steps >= max_steps) {
                continue;
            }

            Pose p = toPose(node.cell, req.x_size);
            for (int dir = 0; dir < kNumDirs; ++dir) {
                const int nx = p.x + kDx[dir];
                const int ny = p.y + kDy[dir];
                if (!isFree(req, nx, ny)) {
                    continue;
                }

                int ncell = toCell(nx, ny, req.x_size);
                long long ng = node.g + cellCost(req, node.cell);
                auto old = best_g.find(ncell);
                if (old != best_g.end() && old->second <= ng) {
                    continue;
                }
                best_g[ncell] = ng;

                Node child;
                child.cell = ncell;
                child.g = ng;
                child.steps = node.steps + 1;
                child.parent = item.node_id;
                int child_id = static_cast<int>(nodes.size());
                nodes.push_back(child);

                double h = static_cast<double>(req.min_free_cost) * chebyshev(nx, ny, goal.x, goal.y);
                open.push(QueueItem{static_cast<double>(ng) + 1.6 * h, ng, child_id});
            }
        }

        return false;
    }
};

}  // namespace stp

#endif  // STP_SPATIAL_INTERCEPT_ASTAR_H
