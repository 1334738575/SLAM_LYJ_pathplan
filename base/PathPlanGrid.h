#pragma once
#include <Eigen/Core>

namespace PATH_PLAN_LYJ
{
    struct PathPlanNode
    {
        /* data */
        PathPlanNode(const Eigen::Vector3i &_loc)
        {
            loc_ = _loc;
        }
        Eigen::Vector3i loc_;
        // bool valid_ = true;
        // double f_ = 0;
        int status = 0; //-1 for obstacle, 0 for default, 1 for open, 2 for close
        double g_ = 0;
        double h_ = 0;
        PathPlanNode *parent_ = nullptr;
        inline double getf() const { return g_ + h_; }
        void calculateH(PathPlanNode *_dst)
        {
            for (int i = 0; i < 3; ++i)
                h_ += std::abs(_dst->loc_(i) - loc_(i));
        }
        static double calculateH(PathPlanNode *_src, PathPlanNode *_dst)
        {
            double h = 0;
            for (int i = 0; i < 3; ++i)
                h += std::abs(_dst->loc_(i) - _src->loc_(i));
            return h;
        }
        void calculateG(PathPlanNode *_src)
        {
            double v = 0;
            for (int i = 0; i < 3; ++i)
                v += (_src->loc_(i) - loc_(i)) * (_src->loc_(i) - loc_(i));
            g_ = _src->g_ + std::sqrt(v);
        }
        static double calculateG(PathPlanNode *_src, PathPlanNode *_dst)
        {
            double v = 0;
            for (int i = 0; i < 3; ++i)
                v += (_src->loc_(i) - _dst->loc_(i)) * (_src->loc_(i) - _dst->loc_(i));
            return _src->g_ + std::sqrt(v);
        }
    };

    class PathPlanGrid
    {
    private:
        /* data */
        Eigen::Vector3d minP_;
        Eigen::Vector3d maxP_;
        Eigen::Vector3i size_;
        double resol_;
        std::vector<PathPlanNode *> nodes_;

    public:
        PathPlanGrid(const Eigen::Vector3d &_minP, const Eigen::Vector3d &_maxP, const double _resolution, const double _rbtRadius, const std::vector<Eigen::Vector3d> &_obstacles);
        ~PathPlanGrid();

        bool real2Grid(const Eigen::Vector3d &_realP, Eigen::Vector3i &_gridP);
        bool grid2Real(const Eigen::Vector3i &_gridP, Eigen::Vector3d &_realP);
        int real2Ind(const Eigen::Vector3d &_realP);

        PathPlanNode *getNode(const Eigen::Vector3i &_gridP);
        PathPlanNode *getNode(const Eigen::Vector3d &_readP);
        PathPlanNode *checkNode(const int &_ind);

        void getAroundNodes(PathPlanNode *_node, std::vector<PathPlanNode *> &_nodes);

    private:
        bool checkP(const Eigen::Vector3d &_realP);
        bool checkP(const Eigen::Vector3i &_gridP);
        int grid2Ind(const Eigen::Vector3i &_gridP);
    };

} // namespace PATH_PLAN_LYJ
