#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    /*
    3D diagonal (26-connected) distance heuristic in grid-index space.

    Weights:
        - axis-aligned moves with cost 1,
        - face-diagonal moves with cost sqrt(2),
        - space-diagonal moves with cost sqrt(3).

    Computes the heuristic by:
        1) taking as many 3D diagonal steps as possible,
        2) then 2D diagonal steps in the remaining plane,
        3) then axis-aligned steps.
    */

    int dx = abs(node1->index(0) - node2->index(0));
    int dy = abs(node1->index(1) - node2->index(1));
    int dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    else if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    else if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    /*
    Manhattan (L1) distance heuristic. Assumes unit cost for axis moves and ignores diagonals.

    The heuristic is computed by taking only axis-aligned steps. 
    */
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    /*
    Euclidian (L2) distance heuristic. Assumes unit cost for axis moves.

    The heuristic is computed by treating the grid as a continuous 3D space and calculating the straight-line distance between the two nodes.
    */
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        // RCLCPP_WARN(rclcpp::get_logger("ConvertToIndexAndAdjustStartEndPoints"), "Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        // RCLCPP_WARN(rclcpp::get_logger("ConvertToIndexAndAdjustStartEndPoints"), "End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    /*
    Perform A* search on a 3D voxel grid to find a collision-free path
    between a start point and a goal point.

    The search is performed in grid-index space using a 26-connected
    neighborhood (axis-aligned, face-diagonal, and space-diagonal moves).
    Edge costs correspond to the Euclidean length of each grid move, and
    the heuristic is provided by getHeu().

    The algorithm terminates when the goal grid cell is selected for
    expansion, at which point the optimal path is reconstructed by
    following parent pointers. A strict time limit is enforced to ensure
    real-time performance.

    Inputs:
        - step_size:
            Resolution of the voxel grid (size of each grid cell).
        - start_pt:
            Start position in world coordinates.
        - end_pt:
            Goal position in world coordinates.

    Returns:
        - true  if a collision-free path to the goal is found within the time limit.
        - false if no path is found or the search exceeds the time budget.
    */

    rclcpp::Time time_1 = rclcpp::Clock().now();
    ++rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        RCLCPP_ERROR(rclcpp::get_logger("AstarSearch"), "Unable to handle the initial or end point, force return!");
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<OpenItem, std::vector<OpenItem>, OpenItemComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push({startPtr->fScore, startPtr}); //put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;

        // Select node with lowest fScore from open set
        GridNodePtr current = nullptr;

        while (!openSet_.empty()) {
            auto [f_at_push, node] = openSet_.top();
            openSet_.pop();

            // Skip entries from previous rounds (extra safety)
            if (node->rounds != rounds_) continue;

            // Skip nodes already closed
            if (node->state == GridNode::CLOSEDSET) continue;

            // Skip stale priority entries (node fscore was improved or worsened after this entry was pushed)
            if (std::abs(f_at_push - node->fScore) > 1e-12) continue;

            current = node;
            break;
        }

        if (!current) break; // nothing usable left


        // Goal reached: reconstruct path.
        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        // Explore 26-connected neighborhood
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    // Skip the current node itself
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;
                    
                    // Bounds check (avoid accessing invalid grid cells)
                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || 
                        neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || 
                        neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    // Frist, check if neighbor voxel is free (continue directly if occupied)
                    if (checkOccupancy(Index2Coord(neighborIdx)))
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    // Check whether this node has already been explored in this round
                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    // Cost of moving from current to neighbor
                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push({neighborPtr->fScore, neighborPtr});  //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { 
                        // Found a better path to an already discovered node
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        // Re-push to "update" priority in std::priority_queue
                        // Otherwise queue in openSet_ is wrong, since it does not automatically update the prio
                        openSet_.push({neighborPtr->fScore, neighborPtr});
                    }
                }

        // Enforce a strict time limit to maintain real-time performance
        rclcpp::Time time_2 = rclcpp::Clock().now();
        if ((time_2 - time_1).seconds() > 0.2)
        {
            RCLCPP_WARN(rclcpp::get_logger("AstarSearch"),
                        "A* search failed: 0.2s time limit exceeded.");
            return false;
        }
    }

    // Search exhausted without reaching the goal
    rclcpp::Time time_2 = rclcpp::Clock().now();

    if ((time_2 - time_1).seconds() > 0.1){
        RCLCPP_WARN(
            rclcpp::get_logger("AstarSearch"),
            "A* search terminated: no feasible path found "
            "(time=%.3fs, iterations=%d)",
            (time_2 - time_1).seconds(), num_iter);
        }

    return false;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
