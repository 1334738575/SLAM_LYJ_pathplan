#pragma once

#include "PathPlanningDefines.h"

namespace PATH_PLAN_LYJ
{

    PATH_PLAN_LYJ_API PathPlanHandle createPathPlanner(
        const Eigen::Vector3d &_minP, const Eigen::Vector3d &_maxP,
        const double _resolution, const double _rbtRadius,
        const std::vector<Eigen::Vector3d> &_obstacles,
        PathPlanParam _param = PathPlanParam());

    PATH_PLAN_LYJ_API bool planTwoLocations(
        PathPlanHandle _handle,
        const Eigen::Vector3d &_src, const Eigen::Vector3d &_dst,
        std::vector<Eigen::Vector3d> &_path);

    PATH_PLAN_LYJ_API void releasePathPlanner(PathPlanHandle _handle);

}