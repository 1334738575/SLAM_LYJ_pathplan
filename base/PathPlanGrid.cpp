#include "PathPlanGrid.h"
#include <fstream>
#include <iostream>

namespace PATH_PLAN_LYJ
{

    PathPlanGrid::PathPlanGrid(const Eigen::Vector3d &_minP, const Eigen::Vector3d &_maxP, const double _resolution, const double _rbtRadius, const std::vector<Eigen::Vector3d> &_obstacles)
        : minP_(_minP), maxP_(_maxP), resol_(_resolution)
    {
        for (int i = 0; i < 3; ++i)
            size_(i) = (maxP_(i) - minP_(i)) / resol_;
        nodes_.resize(size_(0) * size_(1) * size_(2), nullptr);
        Eigen::Vector3d obtMin;
        Eigen::Vector3d obtMax;
        Eigen::Vector3i obtGridMin;
        Eigen::Vector3i obtGridMax;
        Eigen::Vector3i gridPTmp;
        int indTmp;
        for (int oi = 0; oi < _obstacles.size(); ++oi)
        {
            //gridPTmp
            if (_obstacles[oi](0) < minP_(0) || _obstacles[oi](1) < minP_(1) || _obstacles[oi](2) < minP_(2) ||
                _obstacles[oi](0) > maxP_(0) || _obstacles[oi](1) > maxP_(1) || _obstacles[oi](2) > maxP_(2)) {
                std::cout << "out of grid " << std::endl;
                continue; // obstacle is out of the grid
            }
            for (int ri = 0; ri < 3; ++ri)
            {
                obtGridMin(ri) = (_obstacles[oi](ri) - _rbtRadius - minP_(ri)) / resol_;
                obtGridMax(ri) = (_obstacles[oi](ri) + _rbtRadius - minP_(ri)) / resol_;
            }
            for (int i = obtGridMin(0); i <= obtGridMax(0); ++i)
            {
                gridPTmp(0) = i;
                for (int j = obtGridMin(1); j <= obtGridMax(1); ++j)
                {
                    gridPTmp(1) = j;
                    for (int k = obtGridMin(2); k <= obtGridMax(2); ++k)
                    {

                        gridPTmp(2) = k;
                        if (!checkP(gridPTmp))
                            continue;
                        indTmp = grid2Ind(gridPTmp);
                        if (nodes_[indTmp] == nullptr)
                            nodes_[indTmp] = new PathPlanNode(gridPTmp);
                        nodes_[indTmp]->status = -1;
                    }
                }
            }
        }
    }
    PathPlanGrid::~PathPlanGrid()
    {
        if (debugPath_ != "")
        {
            std::ofstream f(debugPath_ + "/PathPlanGrid.txt");
            for (int i = 0; i < nodes_.size(); ++i)
            {
                if (nodes_[i] == nullptr)
                    continue;
                if (nodes_[i]->status == -1)
                    f << nodes_[i]->loc_(0) << " " << nodes_[i]->loc_(1) << " " << nodes_[i]->loc_(2) << " " << 0 << " " << 0 << " " << 0 << std::endl;
                else if (nodes_[i]->status == 0)
                    f << nodes_[i]->loc_(0) << " " << nodes_[i]->loc_(1) << " " << nodes_[i]->loc_(2) << " " << 255 << " " << 255 << " " << 255 << std::endl;
                else if (nodes_[i]->status == 1)
                    f << nodes_[i]->loc_(0) << " " << nodes_[i]->loc_(1) << " " << nodes_[i]->loc_(2) << " " << 255 << " " << 255 << " " << 0 << std::endl;
                else if (nodes_[i]->status == 2)
                    f << nodes_[i]->loc_(0) << " " << nodes_[i]->loc_(1) << " " << nodes_[i]->loc_(2) << " " << 0 << " " << 255 << " " << 0 << std::endl;
            }
            f.close();
        }
        for (int i = 0; i < nodes_.size(); ++i)
        {
            if (nodes_[i])
                delete nodes_[i];
            nodes_[i] = nullptr;
        }
    }

    bool PathPlanGrid::real2Grid(const Eigen::Vector3d &_realP, Eigen::Vector3i &_gridP)
    {
        if (!checkP(_realP))
            return false;
        Eigen::Vector3d relP = _realP - minP_;
        for (int i = 0; i < 3; ++i)
            _gridP(i) = relP(i) / resol_;
        return true;
    }
    bool PathPlanGrid::grid2Real(const Eigen::Vector3i &_gridP, Eigen::Vector3d &_realP)
    {
        if (!checkP(_gridP))
            return false;
        for (int i = 0; i < 3; ++i)
            _realP(i) = minP_(i) + resol_ * _gridP(i);
        // _realP(i) = minP_(i) + resol_ * (_gridP(i) + 0.5);
        return true;
    }
    int PathPlanGrid::real2Ind(const Eigen::Vector3d &_realP)
    {
        Eigen::Vector3i gridP;
        if (!real2Grid(_realP, gridP))
            return -1;
        return grid2Ind(gridP);
    }
    PathPlanNode *PathPlanGrid::getNode(const Eigen::Vector3i &_gridP)
    {
        if (!checkP(_gridP))
            return nullptr;
        int ind = grid2Ind(_gridP);
        if (nodes_[ind] == nullptr)
            nodes_[ind] = new PathPlanNode(_gridP);
        return nodes_[ind];
    }
    PathPlanNode *PathPlanGrid::getNode(const Eigen::Vector3d &_readP)
    {
        Eigen::Vector3i gridP;
        if (!real2Grid(_readP, gridP))
            return nullptr;
        return getNode(gridP);
    }
    PathPlanNode *PathPlanGrid::checkNode(const int &_ind)
    {
        return nodes_[_ind];
    }
    void PathPlanGrid::getAroundNodes(PathPlanNode *_node, std::vector<PathPlanNode *> &_nodes)
    {
        _nodes.clear();
        const Eigen::Vector3i gridP = _node->loc_;
        int a = 1;
        Eigen::Vector3i gridPTmp;
        int indTmp;
        //for (int i = -a; i <= a; ++i)
        //{
        //    gridPTmp(0) = gridP(0) + i;
        //    for (int j = -a; j <= a; ++j)
        //    {
        //        gridPTmp(1) = gridP(1) + j;
        //        for (int k = -a; k <= a; ++k)
        //        {
        //            if (i == 0 && j == 0 && k == 0)
        //                continue;
        //            gridPTmp(2) = gridP(2) + k;
        //            if (!checkP(gridPTmp))
        //                continue;
        //            indTmp = grid2Ind(gridPTmp);
        //            if (nodes_[indTmp] == nullptr)
        //                nodes_[indTmp] = new PathPlanNode(gridPTmp);
        //            _nodes.push_back(nodes_[indTmp]);
        //        }
        //    }
        //}
        for (int i = 0; i <= 3; ++i)
        {
            for (int j = -1; j < 2; ++j) {
                gridPTmp = gridP;
                gridPTmp(i) = gridP(i) + j;
                if (!checkP(gridPTmp))
                    continue;
                indTmp = grid2Ind(gridPTmp);
                if (nodes_[indTmp] == nullptr)
                    nodes_[indTmp] = new PathPlanNode(gridPTmp);
                _nodes.push_back(nodes_[indTmp]);
            }
        }
        return;
    }

    bool PathPlanGrid::checkP(const Eigen::Vector3d &_realP)
    {
        if (_realP(0) < minP_(0) || _realP(1) < minP_(1) || _realP(2) < minP_(2) || _realP(0) > maxP_(0) || _realP(1) > maxP_(1) || _realP(2) > maxP_(2))
            return false;
        return true;
    }
    bool PathPlanGrid::checkP(const Eigen::Vector3i &_gridP)
    {
        if (_gridP(0) < 0 || _gridP(1) < 0 || _gridP(2) < 0)
            return false;
        if (_gridP(0) >= size_(0) || _gridP(1) >= size_(1) || _gridP(2) >= size_(2))
            return false;
        return true;
    }
    int PathPlanGrid::grid2Ind(const Eigen::Vector3i &_gridP)
    {
        return _gridP(2) * size_(0) * size_(1) + _gridP(1) * size_(0) + _gridP(0);
    }
} // namespace PATH_PLAN_LYJ
