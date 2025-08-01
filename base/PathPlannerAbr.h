#pragma once
#include <Eigen/Core>

namespace PATH_PLAN_LYJ
{
    class PathPlannerAbr
    {
    private:
        /* data */
    public:
        PathPlannerAbr(/* args */) {};
        ~PathPlannerAbr() {};

        virtual bool planTwoLocations(const Eigen::Vector3d &_src, const Eigen::Vector3d &_dst, std::vector<Eigen::Vector3d> &_path) = 0;
        virtual bool planLocations(const std::vector<Eigen::Vector3d> &_locs, std::vector<Eigen::Vector3d> &_path) = 0;
        virtual void release() = 0;
    };

}