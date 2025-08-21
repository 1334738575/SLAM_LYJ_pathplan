#include "AStar.h"
#include "base/PathPlanGrid.h"

namespace PATH_PLAN_LYJ
{

    PathPlannerAStar::PathPlannerAStar(const Eigen::Vector3d &_minP, const Eigen::Vector3d &_maxP, const double _resolution, const double _rbtRadius, const std::vector<Eigen::Vector3d> &_obstacles, PathPlanParam _param)
        : PathPlannerAbr()
    {
        grid_ = std::make_shared<PathPlanGrid>(_minP, _maxP, _resolution, _rbtRadius, _obstacles);
        param_ = _param;
        grid_->debugPath_ = param_.debugPath;
        grid_->debugOut();
    }
    PathPlannerAStar::~PathPlannerAStar()
    {
    }

    bool PathPlannerAStar::planTwoLocations(const Eigen::Vector3d &_src, const Eigen::Vector3d &_dst, std::vector<Eigen::Vector3d> &_path)
    {
        _path.clear();
        std::vector<PathPlanNode *> openSet;
        std::vector<PathPlanNode *> closeSet;
        PathPlanNode *nodeSrc = grid_->getNode(_src);
        if (nodeSrc == nullptr)
            return false;
        Eigen::Vector3i gridDst;
        if (!grid_->real2Grid(_dst, gridDst))
            return false;
        int indDst = grid_->real2Ind(_dst);
        PathPlanNode *nodeDst = grid_->checkNode(indDst);
        if (nodeDst && nodeDst->status == 2)
            return false;
        nodeDst = new PathPlanNode(gridDst);
        nodeSrc->calculateH(nodeDst);
        openSet.push_back(nodeSrc);
        PathPlanNode *nodeTmp = nullptr;
        std::vector<PathPlanNode *> nodesTmp;
        double gTmp;
        while (!openSet.empty() && grid_->checkNode(indDst) == nullptr)
        {
            std::sort(openSet.begin(), openSet.end(), [](const PathPlanNode *_n1, const PathPlanNode *_n2)
                      { return _n1->getf() > _n2->getf(); });
            nodeTmp = openSet.back();
            openSet.pop_back();
            nodeTmp->status = 2;
            closeSet.push_back(nodeTmp);
            grid_->getAroundNodes(nodeTmp, nodesTmp);
            for (int i = 0; i < nodesTmp.size(); ++i)
            {
                auto &nnn = nodesTmp[i];
                if (nnn->status == -1 || nnn->status == 2)
                    continue;
                else
                {
                    if (nnn->status == 0)
                    {
                        nnn->calculateH(nodeDst);
                        nnn->calculateG(nodeTmp);
                        nnn->parent_ = nodeTmp;
                        nnn->status = 1;
                        openSet.push_back(nnn);
                    }
                    else if (nnn->status == 1)
                    {
                        gTmp = PathPlanNode::calculateG(nodeTmp, nnn);
                        if (gTmp < nnn->g_)
                        {
                            nnn->g_ = gTmp;
                            nnn->parent_ = nodeTmp;
                        }
                    }
                }
            }
        }
        nodeTmp = grid_->checkNode(indDst);
        Eigen::Vector3d realTmp;
        if (nodeTmp != nullptr)
        {
            while (nodeTmp != nullptr)
            {
                grid_->grid2Real(nodeTmp->loc_, realTmp);
                _path.push_back(realTmp);
                nodeTmp = nodeTmp->parent_;
            }
            _path.front() = _dst;
            _path.back() = _src;
            std::reverse(_path.begin(), _path.end());
        }
        delete nodeDst;
        grid_->reset();
        return true;
    }
    bool PathPlannerAStar::planLocations(const std::vector<Eigen::Vector3d> &_locs, std::vector<Eigen::Vector3d> &_path)
    {
        return false;
    }
    void PathPlannerAStar::release()
    {
        grid_ = nullptr;
    }
}