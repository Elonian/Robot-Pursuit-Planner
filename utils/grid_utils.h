#ifndef STP_GRID_UTILS_H
#define STP_GRID_UTILS_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace stp {

constexpr int kNumDirs = 9;
const int kDx[kNumDirs] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
const int kDy[kNumDirs] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
constexpr long long kInfCost = std::numeric_limits<long long>::max() / 4;

struct Pose {
    int x = 0;
    int y = 0;
};

struct PlanningRequest {
    const int* map = nullptr;
    int collision_thresh = 0;
    int x_size = 0;
    int y_size = 0;
    int robot_x = 0;
    int robot_y = 0;
    int target_steps = 0;
    const int* target_traj = nullptr;
    int curr_time = 0;
    int min_free_cost = 1;
};

struct TimedPlan {
    int start_time = -1;
    std::vector<Pose> poses;
};

inline int mapIndex(int x, int y, int x_size) {
    return (y - 1) * x_size + (x - 1);
}

inline int toCell(int x, int y, int x_size) {
    return mapIndex(x, y, x_size);
}

inline Pose toPose(int cell, int x_size) {
    return Pose{cell % x_size + 1, cell / x_size + 1};
}

inline bool inBounds(int x, int y, int x_size, int y_size) {
    return x >= 1 && x <= x_size && y >= 1 && y <= y_size;
}

inline bool isFree(const PlanningRequest& req, int x, int y) {
    if (!inBounds(x, y, req.x_size, req.y_size)) {
        return false;
    }
    int cost = req.map[mapIndex(x, y, req.x_size)];
    return cost >= 0 && cost < req.collision_thresh;
}

inline bool isFreeCell(const PlanningRequest& req, int cell) {
    int cost = req.map[cell];
    return cost >= 0 && cost < req.collision_thresh;
}

inline int cellCost(const PlanningRequest& req, int cell) {
    return std::max(1, req.map[cell]);
}

inline int chebyshev(int x0, int y0, int x1, int y1) {
    return std::max(std::abs(x0 - x1), std::abs(y0 - y1));
}

inline Pose targetAt(const PlanningRequest& req, int t) {
    t = std::max(0, std::min(t, req.target_steps - 1));
    return Pose{req.target_traj[t], req.target_traj[t + req.target_steps]};
}

inline int targetCellAt(const PlanningRequest& req, int t) {
    Pose p = targetAt(req, t);
    return toCell(p.x, p.y, req.x_size);
}

inline uint64_t spacetimeKey(int cell, int t) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(t)) << 32) |
           static_cast<uint32_t>(cell);
}

inline double elapsedMs(std::chrono::steady_clock::time_point start) {
    using namespace std::chrono;
    return duration_cast<duration<double, std::milli>>(steady_clock::now() - start).count();
}

inline int computeMinFreeCost(const int* map, int collision_thresh, int cell_count) {
    int min_cost = std::numeric_limits<int>::max();
    for (int i = 0; i < cell_count; ++i) {
        if (map[i] >= 0 && map[i] < collision_thresh) {
            min_cost = std::min(min_cost, std::max(1, map[i]));
        }
    }
    return min_cost == std::numeric_limits<int>::max() ? 1 : min_cost;
}

inline bool validOneStepAction(const PlanningRequest& req, Pose next) {
    return isFree(req, next.x, next.y) &&
           std::abs(next.x - req.robot_x) <= 1 &&
           std::abs(next.y - req.robot_y) <= 1;
}

inline long long scorePathWithEvaluatorCost(const PlanningRequest& req, const std::vector<Pose>& path) {
    long long cost = 0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        cost += cellCost(req, toCell(path[i].x, path[i].y, req.x_size));
    }
    return cost;
}

}  // namespace stp

#endif  // STP_GRID_UTILS_H
