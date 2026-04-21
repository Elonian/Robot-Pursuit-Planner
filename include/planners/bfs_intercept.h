#ifndef STP_BFS_INTERCEPT_H
#define STP_BFS_INTERCEPT_H

#include "../../utils/grid_utils.h"

#include <algorithm>
#include <chrono>
#include <unordered_map>
#include <vector>

namespace stp {

class BfsInterceptPlanner {
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

        std::unordered_map<int, std::vector<int> > target_times;
        target_times.reserve(static_cast<size_t>(req.target_steps - req.curr_time));
        for (int t = req.curr_time + 1; t < req.target_steps; ++t) {
            int cell = targetCellAt(req, t);
            if (isFreeCell(req, cell)) {
                target_times[cell].push_back(t);
            }
        }
        if (target_times.empty()) {
            return false;
        }

        const int cell_count = req.x_size * req.y_size;
        const int start_cell = toCell(req.robot_x, req.robot_y, req.x_size);
        std::vector<int> dist(static_cast<size_t>(cell_count), -1);
        std::vector<int> parent(static_cast<size_t>(cell_count), -1);
        std::vector<int> queue;
        queue.reserve(std::min(cell_count, 250000));

        dist[start_cell] = 0;
        queue.push_back(start_cell);
        size_t head = 0;

        int goal_cell = -1;
        int intercept_time = -1;
        while (head < queue.size()) {
            if ((head & 4095U) == 0U && elapsedMs(start_time) > budget_ms) {
                return false;
            }

            int cell = queue[head++];
            int arrival_time = req.curr_time + dist[cell];
            auto target_it = target_times.find(cell);
            if (target_it != target_times.end()) {
                const std::vector<int>& times = target_it->second;
                auto lb = std::lower_bound(times.begin(), times.end(), arrival_time);
                if (lb != times.end()) {
                    goal_cell = cell;
                    intercept_time = *lb;
                    break;
                }
            }

            if (arrival_time >= req.target_steps - 1) {
                continue;
            }

            Pose p = toPose(cell, req.x_size);
            for (int dir = 0; dir < kNumDirs; ++dir) {
                int nx = p.x + kDx[dir];
                int ny = p.y + kDy[dir];
                if (!isFree(req, nx, ny)) {
                    continue;
                }
                int next_cell = toCell(nx, ny, req.x_size);
                if (dist[next_cell] >= 0) {
                    continue;
                }
                dist[next_cell] = dist[cell] + 1;
                parent[next_cell] = cell;
                queue.push_back(next_cell);
            }
        }

        if (goal_cell < 0 || intercept_time < 0) {
            return false;
        }

        std::vector<Pose> reversed;
        int cur = goal_cell;
        while (cur >= 0) {
            reversed.push_back(toPose(cur, req.x_size));
            cur = parent[cur];
        }
        out_path.assign(reversed.rbegin(), reversed.rend());
        while (static_cast<int>(out_path.size()) < intercept_time - req.curr_time + 1) {
            out_path.push_back(out_path.back());
        }
        return out_path.size() >= 2;
    }
};

}  // namespace stp

#endif  // STP_BFS_INTERCEPT_H
