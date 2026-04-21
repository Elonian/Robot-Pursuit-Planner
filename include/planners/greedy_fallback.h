#ifndef STP_GREEDY_FALLBACK_H
#define STP_GREEDY_FALLBACK_H

#include "../../utils/grid_utils.h"

namespace stp {

class GreedyFallbackPlanner {
public:
    Pose nextAction(const PlanningRequest& req) const {
        int look_time = std::min(req.target_steps - 1, req.curr_time + 1);
        Pose target = targetAt(req, look_time);

        Pose best{req.robot_x, req.robot_y};
        long long best_score = kInfCost;
        for (int dir = 0; dir < kNumDirs; ++dir) {
            int nx = req.robot_x + kDx[dir];
            int ny = req.robot_y + kDy[dir];
            if (!isFree(req, nx, ny)) {
                continue;
            }
            long long dist = std::abs(nx - target.x) + std::abs(ny - target.y);
            long long score = 1000000LL * dist + cellCost(req, toCell(nx, ny, req.x_size));
            if (score < best_score) {
                best_score = score;
                best = Pose{nx, ny};
            }
        }
        return best;
    }
};

}  // namespace stp

#endif  // STP_GREEDY_FALLBACK_H
