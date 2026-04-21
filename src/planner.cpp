/*=================================================================
 *
 * planner.cpp
 *
 * Required entrypoint for the homework harness.  The implementation is
 * intentionally thin: reusable grid utilities live in utils/, and individual
 * planning strategies live in include/planners/.
 *
 *=================================================================*/
#include "../include/planner.h"

#include "../include/planners/hybrid_planner.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>

namespace {

struct PlannerCache {
    const int* map = nullptr;
    const int* target_traj = nullptr;
    int collision_thresh = -1;
    int x_size = 0;
    int y_size = 0;
    int target_steps = 0;
    int min_free_cost = 1;
    stp::TimedPlan plan;

    bool sameProblem(
        const int* map_ptr,
        int collision,
        int xs,
        int ys,
        int steps,
        const int* traj_ptr) const {
        return map == map_ptr && target_traj == traj_ptr &&
               collision_thresh == collision && x_size == xs &&
               y_size == ys && target_steps == steps;
    }

    void resetProblem(
        const int* map_ptr,
        int collision,
        int xs,
        int ys,
        int steps,
        const int* traj_ptr) {
        map = map_ptr;
        target_traj = traj_ptr;
        collision_thresh = collision;
        x_size = xs;
        y_size = ys;
        target_steps = steps;
        min_free_cost = stp::computeMinFreeCost(map, collision_thresh, x_size * y_size);
        plan.start_time = -1;
        plan.poses.clear();
    }
};

PlannerCache g_cache;

bool cachedActionIsUsable(const stp::PlanningRequest& req, stp::Pose& action) {
    const stp::TimedPlan& plan = g_cache.plan;
    if (plan.start_time < 0 || plan.poses.empty()) {
        return false;
    }

    int offset = req.curr_time - plan.start_time;
    if (offset < 0 || offset >= static_cast<int>(plan.poses.size())) {
        return false;
    }

    stp::Pose expected = plan.poses[offset];
    if (expected.x != req.robot_x || expected.y != req.robot_y) {
        return false;
    }

    int next_offset = std::min(offset + 1, static_cast<int>(plan.poses.size()) - 1);
    action = plan.poses[next_offset];
    return stp::validOneStepAction(req, action);
}

bool runSelectedPlanner(const stp::PlanningRequest& req, stp::TimedPlan& plan) {
    const char* selected = std::getenv("STP_PLANNER");
    if (selected == nullptr || std::strcmp(selected, "hybrid") == 0) {
        stp::HybridPlanner hybrid;
        return hybrid.plan(req, plan);
    }

    plan.start_time = req.curr_time;
    plan.poses.clear();
    auto start_time = std::chrono::steady_clock::now();

    if (std::strcmp(selected, "direct") == 0) {
        stp::DirectInterceptPlanner direct;
        return direct.plan(req, start_time, 900.0, plan.poses);
    }
    if (std::strcmp(selected, "multigoal") == 0 || std::strcmp(selected, "multi_goal") == 0) {
        stp::MultiGoalInterceptAStar multi_goal;
        return multi_goal.plan(req, start_time, 900.0, plan.poses);
    }
    if (std::strcmp(selected, "spatial") == 0) {
        stp::SpatialInterceptAStar spatial;
        return spatial.plan(req, start_time, 900.0, plan.poses);
    }
    if (std::strcmp(selected, "bfs") == 0) {
        stp::BfsInterceptPlanner bfs;
        return bfs.plan(req, start_time, 900.0, plan.poses);
    }
    if (std::strcmp(selected, "spacetime") == 0) {
        stp::SpaceTimeWeightedAStar space_time;
        return space_time.plan(req, start_time, 900.0, plan.poses);
    }
    if (std::strcmp(selected, "greedy") == 0) {
        stp::GreedyFallbackPlanner greedy;
        plan.poses.push_back(stp::Pose{req.robot_x, req.robot_y});
        plan.poses.push_back(greedy.nextAction(req));
        return stp::validOneStepAction(req, plan.poses.back());
    }

    stp::HybridPlanner hybrid;
    return hybrid.plan(req, plan);
}

}  // namespace

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr)
{
    (void)targetposeX;
    (void)targetposeY;

    if (!g_cache.sameProblem(map, collision_thresh, x_size, y_size, target_steps, target_traj)) {
        g_cache.resetProblem(map, collision_thresh, x_size, y_size, target_steps, target_traj);
    }

    stp::PlanningRequest req;
    req.map = map;
    req.collision_thresh = collision_thresh;
    req.x_size = x_size;
    req.y_size = y_size;
    req.robot_x = robotposeX;
    req.robot_y = robotposeY;
    req.target_steps = target_steps;
    req.target_traj = target_traj;
    req.curr_time = curr_time;
    req.min_free_cost = g_cache.min_free_cost;

    stp::Pose action;
    if (!cachedActionIsUsable(req, action)) {
        if (!runSelectedPlanner(req, g_cache.plan) || g_cache.plan.poses.size() < 2) {
            action = stp::Pose{robotposeX, robotposeY};
        } else {
            action = g_cache.plan.poses[1];
        }
    }

    if (!stp::validOneStepAction(req, action)) {
        action = stp::Pose{robotposeX, robotposeY};
    }

    action_ptr[0] = action.x;
    action_ptr[1] = action.y;
}
