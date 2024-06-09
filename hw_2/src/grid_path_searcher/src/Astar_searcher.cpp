#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

    GridNodeMap = new GridNodePtr **[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                Vector3i tmpIdx(i, j, k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;

    int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{
    vector<Vector3d> visited_nodes;
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if (GridNodeMap[i][j][k]->id == -1) // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %zu", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index)
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt)
{
    Vector3i idx;
    idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
        min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
        min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets, vector<double> &edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself
    please write your code below
    *
    *
    */
    /******************************* start *******************************/
    int id_x = currentPtr->index[0];
    int id_y = currentPtr->index[1];
    int id_z = currentPtr->index[2];

    // up down, left right, forward back, we need to judge whether the node is occupied
    std::vector<int> id_xSets(3), id_ySets(3), id_zSets(3);
    id_xSets = {-1, 0, 1};
    id_ySets = {-1, 0, 1};
    id_zSets = {-1, 0, 1};

    Vector3i neighbour_idx(3);
    Vector3d neighbour_pt(3);

    for (int dx : id_xSets)
    {
        for (int dy : id_ySets)
        {

            for (int dz : id_zSets)
            {
                // update the target index
                int newx = id_x + dx, newy = id_y + dy, newz = id_z + dz;

                // make sure the target is not itself, within boundary, and is not obstacle
                bool is_self = dx == 0 && dy == 0 && dz == 0;
                bool is_boundary = (newx < 0) || (newx >= GLX_SIZE) || (newy < 0) || (newy >= GLY_SIZE) || (newz < 0) || (newz >= GLZ_SIZE);

                if (is_self || is_boundary)
                    continue;

                // if the condition above is satisfied, push it into the neighbourPtrSets, and edgeCostSets
                // update the camefrom node, update the idx
                GridNodePtr gridPtr = GridNodeMap[newx][newy][newz];

                edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
                neighborPtrSets.push_back(gridPtr);
            }
        }
    }

    /******************************* end *******************************/
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /*
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */

    /******************************* start *******************************/

    enum Function
    {
        MANHATTAN,
        EUCLIDEAN,
        DIAGNAL,
        DIJKSTRA
    };

    // define the heuristic function
    double distance = 0.0, fScore = 0.0;
    double dx, dy, dz, sum;

    Function heufunc = DIAGNAL;
    switch (heufunc)
    {
    case MANHATTAN:
        distance = (node1->coord - node2->coord).cwiseAbs().sum();
        break;

    case EUCLIDEAN:
        distance = (node1->coord - node2->coord).norm();
        break;

    case DIAGNAL:
        dx = abs(node1->coord[0] - node2->coord[0]);
        dy = abs(node1->coord[1] - node2->coord[1]);
        dz = abs(node1->coord[2] - node2->coord[2]);
        sum = dx + dy + dz;
        distance = sum + (sqrt(3) - 3) * min(min(dx, dy), dz) + (sqrt(2) - 2) * (sum - min(min(dx, dy), dz) - max(max(dx, dy), dz));
        break;

    default:
        distance = 0.0;
    }

    // update f score
    /**
     * f score has two part:
     * 1. g score -> alpha
     * 2. h score -> epsilon
     * each part has a coefficient which indicates the greedy degree
     */
    double alpha = 1, epsilon = 1;
    fScore = alpha * node1->gScore + epsilon * distance;

    return fScore;
    /******************************* end *******************************/
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();

    // index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    // position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    // Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr = new GridNode(end_idx, end_pt);

    // openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;

    // put start node in open set
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    // STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr->id = 1;
    startPtr->coord = start_pt;
    openSet.insert(make_pair(startPtr->fScore, startPtr));
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    /******************************* start *******************************/
    GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]] = startPtr;

    /******************************* end *******************************/
    // this is the main loop
    while (!openSet.empty())
    {
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below

        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */

        /******************************* start *******************************/
        currentPtr = openSet.begin()->second;
        currentPtr->id = -1;            // mark it as closed
        openSet.erase(openSet.begin()); // remove it from priority queue
        /******************************* end *******************************/

        // if the current node is the goal
        if (currentPtr->index == goalIdx)
        {
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution);
            return;
        }
        // get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets); // STEP 4: finish AstarPathFinder::AstarGetSucc yourself

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *
        */
        /******************************* start *******************************/

        /******************************* end *******************************/
        for (int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below

            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *
            */

            /******************************* start *******************************/
            // please note in every GrideNode, there is a nodeMapIt function, I think I have to make full use of that
            neighborPtr = neighborPtrSets[i];
            // judge whether is's a obstracle, if there's a obstracle, ignore it
            if (isOccupied(neighborPtr->index))
                continue;
            /******************************* end *******************************/

            if (neighborPtr->id == 0)
            { // discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *
                */
                /******************************* start *******************************/
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = getHeu(neighborPtr, endPtr);
                // insert it into the priority queue
                neighborPtr->id = 1;
                neighborPtr->cameFrom = currentPtr;
                openSet.insert(std::make_pair(neighborPtr->fScore, neighborPtr));
                /******************************* end *******************************/
                continue;
            }
            else if (neighborPtr->id == 1 and neighborPtr->gScore > currentPtr->gScore + edgeCostSets[i])
            { // this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *
                */
                /******************************* start *******************************/
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = getHeu(neighborPtr, endPtr);
                neighborPtr->cameFrom = currentPtr;
                /******************************* end *******************************/
                continue;
            }
            else
            { // this node is in closed set
                /*
                *
                please write your code below
                *
                */
                /******************************* start *******************************/

                /******************************* end *******************************/
                continue;
            }
        }
    }
    // if search fails
    ros::Time time_2 = ros::Time::now();
    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec());
}

vector<Vector3d> AstarPathFinder::getPath()
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *
    */
    /******************************* start *******************************/
    GridNodePtr ptr = terminatePtr;
    while (ptr != NULL)
    {
        path.push_back(ptr->coord);
        gridPath.push_back(ptr);
        // std::cout << "path" << ptr->coord << std::endl;
        std::cout << "terminal: " << terminatePtr->index.transpose() << std::endl;
        std::cout << "index: " << ptr->index.transpose() << std::endl;
        ptr = ptr->cameFrom;
    }

    /******************************* end *******************************/
    for (auto ptr : gridPath)
        path.push_back(ptr->coord);

    reverse(path.begin(), path.end());

    return path;
}