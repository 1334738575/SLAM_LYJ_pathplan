#pragma once

#include <memory>

#include <Eigen/Core>

#include "base/PathPlannerAbr.h"
#include "base/PathPlanGrid.h"
#include "PathPlanningDefines.h"

namespace PATH_PLAN_LYJ
{
    class PathPlannerAStar : public PathPlannerAbr
    {
    private:
        /* data */
    public:
        PathPlannerAStar(const Eigen::Vector3d &_minP, const Eigen::Vector3d &_maxP, const double _resolution, const double _rbtRadius, const std::vector<Eigen::Vector3d> &_obstacles, PathPlanParam _param);
        ~PathPlannerAStar();

        // inherit form PathPlannerAbr
        virtual bool planTwoLocations(const Eigen::Vector3d &_src, const Eigen::Vector3d &_dst, std::vector<Eigen::Vector3d> &_path) override;
        virtual bool planLocations(const std::vector<Eigen::Vector3d> &_locs, std::vector<Eigen::Vector3d> &_path) override;
        virtual void release() override;

    private:
        std::shared_ptr<PathPlanGrid> grid_ = nullptr;
        PathPlanParam param_;
    };

}
