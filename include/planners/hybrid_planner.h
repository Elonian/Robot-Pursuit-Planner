#ifndef STP_HYBRID_PLANNER_H
#define STP_HYBRID_PLANNER_H

#include "bfs_intercept.h"
#include "direct_intercept.h"
#include "greedy_fallback.h"
#include "multi_goal_intercept_astar.h"
#include "space_time_astar.h"
#include "spatial_intercept_astar.h"

#include <chrono>
#include <limits>
#include <vector>

namespace stp {

class HybridPlanner {
public:
    bool plan(const PlanningRequest& req, TimedPlan& plan) const {
        plan.start_time = req.curr_time;
        plan.poses.clear();

        auto start_time = std::chrono::steady_clock::now();
        std::vector<TimedPlan> candidates;

        DirectInterceptPlanner direct_intercept;
        TimedPlan direct_plan;
        direct_plan.start_time = req.curr_time;
        if (direct_intercept.plan(req, start_time, 45.0, direct_plan.poses) &&
            catchesTarget(req, direct_plan)) {
            candidates.push_back(direct_plan);
        }

        const int cell_count = req.x_size * req.y_size;
        const bool small_or_medium = cell_count <= 500000;

        MultiGoalInterceptAStar multi_goal_intercept;
        TimedPlan multi_goal_plan;
        multi_goal_plan.start_time = req.curr_time;
        double multi_goal_budget = small_or_medium ? 1250.0 : 620.0;
        if (multi_goal_intercept.plan(req, start_time, multi_goal_budget, multi_goal_plan.poses) &&
            catchesTarget(req, multi_goal_plan)) {
            candidates.push_back(multi_goal_plan);
        }

        SpatialInterceptAStar spatial_intercept;
        TimedPlan spatial_plan;
        spatial_plan.start_time = req.curr_time;
        double spatial_budget = small_or_medium ? 950.0 : 260.0;
        if (spatial_intercept.plan(req, start_time, spatial_budget, spatial_plan.poses) &&
            catchesTarget(req, spatial_plan)) {
            candidates.push_back(spatial_plan);
        }

        if (small_or_medium || candidates.empty()) {
            BfsInterceptPlanner bfs_intercept;
            TimedPlan bfs_plan;
            bfs_plan.start_time = req.curr_time;
            if (bfs_intercept.plan(req, start_time, small_or_medium ? 1400.0 : 900.0, bfs_plan.poses) &&
                catchesTarget(req, bfs_plan)) {
                candidates.push_back(bfs_plan);
            }
        }

        if (!candidates.empty()) {
            plan = chooseLowestCost(req, candidates);
            return plan.poses.size() >= 2;
        }

        SpaceTimeWeightedAStar space_time;
        if (space_time.plan(req, start_time, 930.0, plan.poses)) {
            return catchesTarget(req, plan);
        }

        GreedyFallbackPlanner greedy;
        plan.poses.push_back(Pose{req.robot_x, req.robot_y});
        plan.poses.push_back(greedy.nextAction(req));
        return validOneStepAction(req, plan.poses.back());
    }

private:
    static bool catchesTarget(const PlanningRequest& req, const TimedPlan& plan) {
        if (plan.poses.empty()) {
            return false;
        }
        int final_time = plan.start_time + static_cast<int>(plan.poses.size()) - 1;
        if (final_time < 0 || final_time >= req.target_steps) {
            return false;
        }
        Pose target = targetAt(req, final_time);
        Pose final_pose = plan.poses.back();
        return final_pose.x == target.x && final_pose.y == target.y;
    }

    static TimedPlan chooseLowestCost(const PlanningRequest& req, const std::vector<TimedPlan>& candidates) {
        long long best_cost = std::numeric_limits<long long>::max();
        size_t best_idx = 0;
        for (size_t i = 0; i < candidates.size(); ++i) {
            long long cost = scorePathWithEvaluatorCost(req, candidates[i].poses);
            if (cost < best_cost) {
                best_cost = cost;
                best_idx = i;
            }
        }
        return candidates[best_idx];
    }
};

}  // namespace stp

#endif  // STP_HYBRID_PLANNER_H
