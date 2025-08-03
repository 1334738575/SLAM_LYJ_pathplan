#include "PathPlanningInclude.h"
#include "AStar/AStar.h"

namespace PATH_PLAN_LYJ
{
    PATH_PLAN_LYJ_API PathPlanHandle createPathPlanner(const Eigen::Vector3d &_minP, const Eigen::Vector3d &_maxP, const double _resolution, const double _rbtRadius, const std::vector<Eigen::Vector3d> &_obstacles, PathPlanParam _param)
    {
        PathPlannerAbr *planner = nullptr;
        if (_param.method == ASTAR)
            planner = new PathPlannerAStar(_minP, _maxP, _resolution, _rbtRadius, _obstacles, _param);
        return (PathPlanHandle *)planner;
    }
    PATH_PLAN_LYJ_API bool planTwoLocations(PathPlanHandle _handle, const Eigen::Vector3d &_src, const Eigen::Vector3d &_dst, std::vector<Eigen::Vector3d> &_path)
    {
        PathPlannerAbr *planner = (PathPlannerAbr *)_handle;
        return planner->planTwoLocations(_src, _dst, _path);
    }
    PATH_PLAN_LYJ_API void releasePathPlanner(PathPlanHandle _handle)
    {
        PathPlannerAbr *planner = (PathPlannerAbr *)_handle;
        planner->release();
        delete planner;
        _handle = nullptr;
        return;
    }
}