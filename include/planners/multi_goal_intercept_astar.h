#ifndef STP_MULTI_GOAL_INTERCEPT_ASTAR_H
#define STP_MULTI_GOAL_INTERCEPT_ASTAR_H

#include "space_time_astar.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

namespace stp {

class MultiGoalInterceptAStar {
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

        GoalIndex goals = buildGoalIndex(req);
        if (goals.target_times.empty()) {
            return false;
        }

        const int start_cell = toCell(req.robot_x, req.robot_y, req.x_size);
        const int remaining = req.target_steps - req.curr_time;
        const int max_expansions = std::max(80000, std::min(420000, remaining * 500));

        std::vector<Node> nodes;
        nodes.reserve(100000);
        std::priority_queue<QueueItem> open;
        std::unordered_map<int, long long> best_g;
        best_g.reserve(120000);

        Node start;
        start.cell = start_cell;
        start.g = 0;
        start.steps = 0;
        nodes.push_back(start);
        best_g[start_cell] = 0;

        open.push(QueueItem{heuristic(req, goals, start_cell), 0, 0});

        long long best_score = kInfCost;
        int best_node = -1;
        int best_intercept_time = -1;
        int expansions = 0;

        while (!open.empty()) {
            if (++expansions > max_expansions ||
                ((expansions & 2047) == 0 && elapsedMs(start_time) > budget_ms)) {
                break;
            }

            QueueItem item = open.top();
            open.pop();
            if (item.node_id < 0 || item.node_id >= static_cast<int>(nodes.size())) {
                continue;
            }
            const Node node = nodes[item.node_id];
            auto best_it = best_g.find(node.cell);
            if (best_it == best_g.end() || best_it->second != node.g) {
                continue;
            }

            int intercept_time = earliestTargetTimeAtOrAfter(req, goals, node.cell, node.steps);
            if (intercept_time >= 0) {
                long long wait_steps = intercept_time - req.curr_time - node.steps;
                long long total = node.g + wait_steps * static_cast<long long>(cellCost(req, node.cell));
                if (total < best_score) {
                    best_score = total;
                    best_node = item.node_id;
                    best_intercept_time = intercept_time;
                }
            }

            if (node.steps >= remaining - 1) {
                continue;
            }

            Pose p = toPose(node.cell, req.x_size);
            for (int dir = 0; dir < kNumDirs; ++dir) {
                int nx = p.x + kDx[dir];
                int ny = p.y + kDy[dir];
                if (!isFree(req, nx, ny)) {
                    continue;
                }

                int ncell = toCell(nx, ny, req.x_size);
                int nsteps = node.steps + 1;
                long long ng = node.g + cellCost(req, node.cell);
                auto old = best_g.find(ncell);
                if (old != best_g.end() && old->second <= ng) {
                    continue;
                }
                best_g[ncell] = ng;
                int child_id = static_cast<int>(nodes.size());

                Node child;
                child.cell = ncell;
                child.g = ng;
                child.steps = nsteps;
                child.parent = item.node_id;
                nodes.push_back(child);

                double h = heuristic(req, goals, ncell);
                open.push(QueueItem{static_cast<double>(ng) + 1.55 * h, ng, child_id});
            }
        }

        if (best_node < 0 || best_intercept_time < 0) {
            return false;
        }

        out_path = reconstruct(nodes, best_node, req.x_size);
        while (static_cast<int>(out_path.size()) < best_intercept_time - req.curr_time + 1) {
            out_path.push_back(out_path.back());
        }
        return out_path.size() >= 2;
    }

private:
    struct Node {
        int cell = 0;
        long long g = 0;
        int steps = 0;
        int parent = -1;
    };

    struct GoalIndex {
        std::unordered_map<int, std::vector<int> > target_times;
        std::vector<int> sample_cells;
        int min_x = 1;
        int max_x = 1;
        int min_y = 1;
        int max_y = 1;
    };

    static GoalIndex buildGoalIndex(const PlanningRequest& req) {
        GoalIndex goals;
        goals.min_x = req.x_size;
        goals.max_x = 1;
        goals.min_y = req.y_size;
        goals.max_y = 1;

        std::vector<int> unique_cells;
        unique_cells.reserve(static_cast<size_t>(req.target_steps - req.curr_time));
        for (int t = req.curr_time + 1; t < req.target_steps; ++t) {
            Pose target = targetAt(req, t);
            int available = t - req.curr_time;
            if (chebyshev(req.robot_x, req.robot_y, target.x, target.y) > available) {
                continue;
            }
            int cell = toCell(target.x, target.y, req.x_size);
            if (!isFreeCell(req, cell)) {
                continue;
            }
            std::vector<int>& times = goals.target_times[cell];
            if (times.empty()) {
                unique_cells.push_back(cell);
                goals.min_x = std::min(goals.min_x, target.x);
                goals.max_x = std::max(goals.max_x, target.x);
                goals.min_y = std::min(goals.min_y, target.y);
                goals.max_y = std::max(goals.max_y, target.y);
            }
            times.push_back(t);
        }

        if (unique_cells.empty()) {
            return goals;
        }

        goals.sample_cells.reserve(64);
        const size_t first_count = std::min<size_t>(24, unique_cells.size());
        for (size_t i = 0; i < first_count; ++i) {
            goals.sample_cells.push_back(unique_cells[i]);
        }
        if (unique_cells.size() > first_count) {
            for (int i = 0; i < 40; ++i) {
                size_t idx = static_cast<size_t>(
                    std::round(static_cast<double>(i) *
                               static_cast<double>(unique_cells.size() - 1) / 39.0));
                int cell = unique_cells[idx];
                if (std::find(goals.sample_cells.begin(), goals.sample_cells.end(), cell) ==
                    goals.sample_cells.end()) {
                    goals.sample_cells.push_back(cell);
                }
            }
        }
        return goals;
    }

    static double heuristic(const PlanningRequest& req, const GoalIndex& goals, int cell) {
        Pose p = toPose(cell, req.x_size);
        int dx = 0;
        if (p.x < goals.min_x) {
            dx = goals.min_x - p.x;
        } else if (p.x > goals.max_x) {
            dx = p.x - goals.max_x;
        }
        int dy = 0;
        if (p.y < goals.min_y) {
            dy = goals.min_y - p.y;
        } else if (p.y > goals.max_y) {
            dy = p.y - goals.max_y;
        }
        int box_dist = std::max(dx, dy);

        int sample_dist = box_dist;
        if (!goals.sample_cells.empty()) {
            sample_dist = std::numeric_limits<int>::max();
            for (int goal_cell : goals.sample_cells) {
                Pose g = toPose(goal_cell, req.x_size);
                sample_dist = std::min(sample_dist, chebyshev(p.x, p.y, g.x, g.y));
            }
        }

        double blended = 0.35 * static_cast<double>(box_dist) +
                         0.65 * static_cast<double>(sample_dist);
        return static_cast<double>(req.min_free_cost) * blended;
    }

    static int earliestTargetTimeAtOrAfter(
        const PlanningRequest& req,
        const GoalIndex& goals,
        int cell,
        int arrival_steps) {
        auto it = goals.target_times.find(cell);
        if (it == goals.target_times.end()) {
            return -1;
        }

        int earliest = std::max(req.curr_time + 1, req.curr_time + arrival_steps);
        const std::vector<int>& times = it->second;
        auto lb = std::lower_bound(times.begin(), times.end(), earliest);
        return lb == times.end() ? -1 : *lb;
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
};

}  // namespace stp

#endif  // STP_MULTI_GOAL_INTERCEPT_ASTAR_H
