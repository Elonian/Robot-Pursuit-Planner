#ifndef STP_DIRECT_INTERCEPT_H
#define STP_DIRECT_INTERCEPT_H

#include "../../utils/grid_utils.h"

#include <algorithm>
#include <chrono>
#include <vector>

namespace stp {

class DirectInterceptPlanner {
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

        for (int t = req.curr_time + 1; t < req.target_steps; ++t) {
            if (elapsedMs(start_time) > budget_ms) {
                return false;
            }
            Pose goal = targetAt(req, t);
            int available = t - req.curr_time;
            int dist = chebyshev(req.robot_x, req.robot_y, goal.x, goal.y);
            if (dist > available) {
                continue;
            }

            std::vector<Pose> path;
            path.reserve(static_cast<size_t>(available + 1));
            Pose cur{req.robot_x, req.robot_y};
            path.push_back(cur);
            bool ok = true;
            while (cur.x != goal.x || cur.y != goal.y) {
                int sx = sign(goal.x - cur.x);
                int sy = sign(goal.y - cur.y);
                Pose next{cur.x + sx, cur.y + sy};
                if (!isFree(req, next.x, next.y)) {
                    ok = false;
                    break;
                }
                cur = next;
                path.push_back(cur);
            }
            if (!ok) {
                continue;
            }

            while (static_cast<int>(path.size()) < available + 1) {
                path.push_back(goal);
            }
            out_path = path;
            return out_path.size() >= 2;
        }
        return false;
    }

private:
    static int sign(int value) {
        return (value > 0) - (value < 0);
    }
};

}  // namespace stp

#endif  // STP_DIRECT_INTERCEPT_H
