#include <iostream>
#include <PathPlanningInclude.h>
#include <fstream>

int main(int argc, char *argv[])
{
    std::cout << "hello PATH_PLAN_LYJ!" << std::endl;

    Eigen::Vector3d minP(0, 0, 0);
    Eigen::Vector3d maxP(20, 20, 20);
    double resolution = 1;
    double rbtRadius = 0.7;
    std::vector<Eigen::Vector3d> obstacles;
    obstacles.push_back(Eigen::Vector3d(5, 5, 5));
    PATH_PLAN_LYJ::PathPlanParam param;
    param._method = PATH_PLAN_LYJ::ASTAR;
    PATH_PLAN_LYJ::PathPlanHandle handle = PATH_PLAN_LYJ::createPathPlanner(minP, maxP, resolution, rbtRadius, obstacles, param);

    Eigen::Vector3d src(1, 1, 1);
    Eigen::Vector3d dst(10, 10, 10);
    std::vector<Eigen::Vector3d> path;
    if (!PATH_PLAN_LYJ::planTwoLocations(handle, src, dst, path))
        std::cout << "can't find path!" << std::endl;

    std::ofstream f("PathPlanPath.txt");
    for (size_t i = 0; i < path.size(); ++i)
    {
        f << path[i](0) << " " << path[i](1) << " " << path[i](2) << " " << 0 << " " << 255 << " " << 0 << std::endl;
    }
    f.close();

    PATH_PLAN_LYJ::releasePathPlanner(handle);

    return 0;
}