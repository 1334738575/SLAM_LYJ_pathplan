#pragma once

#include <Eigen/Core>

#ifdef WIN32
#ifdef _MSC_VER
#define PATH_PLAN_LYJ_API __declspec(dllexport)
#else
#define PATH_PLAN_LYJ_API
#endif
#else
#define PATH_PLAN_LYJ_API
#endif

namespace PATH_PLAN_LYJ
{
    enum PLANMETHOD
    {
        ASTAR = 0
    };

    typedef void *PathPlanHandle;

    struct PathPlanParam
    {
        /* data */
        PLANMETHOD method = ASTAR;
        std::string debugPath = "";
    };

}