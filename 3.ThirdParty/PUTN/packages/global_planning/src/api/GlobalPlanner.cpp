#include "api/GlobalPlanner.h"
#include "GlobalPlanningNode.h"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <random>

namespace putn
{
namespace planner
{
void PFRRTStar::initWithGoal(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos)
{
    Eigen::Vector2d last_end_pos_2D = mEndPos_2d;
    PlanningState last_planning_state = mPlanningState;
    mCurrIter = 0;
    mCurrTime = 0.0;
    mEndPos_2d = project2plane(end_pos);

    std::shared_ptr<Node> node_origin = fitPlane(start_pos);

    if (node_origin == nullptr)
    {
        mPlanningState = Invalid;
        return;
    }

    Node::Ptr node_target = fitPlane(end_pos);

    mPlanningState = (node_target == nullptr) ? Roll : Global;

    mCloseCheckRecord.clear();

    bool inherit_flag = false;

    switch (mPlanningState)
    {
        case Global:
            switch (last_planning_state)
            {
                case Global:
                    if (last_end_pos_2D == mEndPos_2d && inheritPath(node_origin, Path::Global))
                    {
                        inherit_flag = true;
                        node_target = nullptr;
                    }
                    else
                    {
                        mTargetNode = node_target;
                    }
                    break;
                case WithoutGoal:
                    mTargetNode = node_target;
                    inherit_flag = inheritTree(node_origin);
                    break;
                default:
                    mTargetNode = node_target;
                    break;
            }
            break;
        case Roll:
            switch (last_planning_state)
            {
                case Roll:
                    inherit_flag = (last_end_pos_2D == mEndPos_2d && inheritPath(node_origin, Path::Sub));
                    break;
                case WithoutGoal:
                    inherit_flag = inheritTree(node_origin);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }

    if (!inherit_flag)
    {
        mPath = Path();
        mTree.clear();
        mOriginNode = node_origin;
        mTree.push_back(mOriginNode);
    }

    if (mPlanningState == Roll && last_end_pos_2D != mEndPos_2d) mSubGoalThreshold = 1.0f;

    std::vector<std::shared_ptr<Node>> origin_and_goal{mOriginNode};
    if (mPlanningState == Global) origin_and_goal.push_back(mTargetNode);
    mRclNode->visualizeOriginAndGoal(origin_and_goal);
}

void PFRRTStar::initWithoutGoal(const Eigen::Vector3d& start_pos)
{
    mCurrIter = 0;
    mCurrTime = 0.0;

    mCloseCheckRecord.clear();
    mPath = Path();

    PlanningState last_planning_state = mPlanningState;

    std::shared_ptr<Node> node_origin = fitPlane(start_pos);

    if (node_origin == nullptr)
    {
        mPlanningState = Invalid;
        return;
    }

    mPlanningState = WithoutGoal;

    mTargetNode = nullptr;

    if (last_planning_state != WithoutGoal || !inheritTree(node_origin))
    {
        mTree.clear();
        mOriginNode = node_origin;
        mTree.push_back(mOriginNode);
    }
}

void PFRRTStar::updateNode(Node::Ptr input_node)
{
    if (input_node->parent != nullptr)   // Skip the root node
        input_node->cost = input_node->parent->cost + CostBetweenTwoNode(input_node, input_node->parent);

    closeCheck(input_node);

    // Update by recursion
    for (auto& node : input_node->children) updateNode(node);
}

void PFRRTStar::addInvalidNodes(Node::Ptr& input_node, bool ifdelete, std::vector<Node::Ptr>& invalid_nodes)
{
    if (input_node == nullptr) return;
    bool delete_flag = false;
    if (ifdelete || input_node->plane == nullptr)
        delete_flag = true;
    else
    {
        if (input_node->parent != nullptr)
            delete_flag = !mWorld->collisionFree(input_node, input_node->parent);
        else   // If the root node is input
            delete_flag = !mWorld->isFree(input_node->position);
    }
    if (delete_flag) invalid_nodes.push_back(input_node);
    for (auto& node : input_node->children) addInvalidNodes(node, delete_flag, invalid_nodes);
}

void PFRRTStar::trimTree()
{
    std::vector<Node::Ptr> invalid_nodes;
    addInvalidNodes(mOriginNode, false, invalid_nodes);
    for (auto& node : invalid_nodes)
    {
        if (node->parent != nullptr) deleteChildren(node->parent, node);
        for (std::vector<Node::Ptr>::iterator it = mTree.begin(); it != mTree.end(); ++it)
        {
            if (*it == node)
            {
                mTree.erase(it);
                break;
            }
        }
    }
}

bool PFRRTStar::inheritTree(Node::Ptr new_root)
{
    bool result = false;

    // considering that the update of grid map may affect the result of plane-fitting
    for (auto& node : mTree)
    {
        fitPlane(node);
    }

    trimTree();

    float min_dis = gINF;
    Node::Ptr node_insert = nullptr;

    for (const auto& node : mTree)
    {
        float tmp_dis = EuclideanDistance(node, new_root);
        if (tmp_dis < min_dis && mWorld->collisionFree(node, new_root))
        {
            min_dis = tmp_dis;
            node_insert = node;
        }
    }

    if (node_insert != nullptr)
    {
        result = true;
        Node::Ptr node = node_insert;
        std::vector<Node::Ptr> node_record;
        while (node != nullptr)
        {
            node_record.push_back(node);
            node = node->parent;
        }
        for (size_t i = node_record.size() - 1; i > 0; i--)
        {
            deleteChildren(node_record[i], node_record[i - 1]);
            node_record[i]->parent = node_record[i - 1];
            node_record[i - 1]->children.push_back(node_record[i]);
        }
        new_root->children.push_back(node_insert);
        node_insert->parent = new_root;
        mTree.push_back(new_root);
        updateNode(new_root);
        mOriginNode = new_root;

        std::vector<std::pair<Node::Ptr, float>> neighbor_record;
        findNearNeighbors(mOriginNode, neighbor_record);
        reWire(mOriginNode, neighbor_record);
    }
    return result;
}

bool PFRRTStar::inheritPath(Node::Ptr new_root, Path::Type type)
{
    bool result = false;
    if (mPath.type == type)
    {
        /* copy the path */
        std::vector<Node::Ptr> tmp_nodes;
        for (size_t i = 0; i < mPath.nodes.size(); i++)
        {
            Node::Ptr node_now = mPath.nodes[i];
            tmp_nodes.push_back(fitPlane(node_now->plane->init_coord));
            if (tmp_nodes[i] == nullptr ||
                (tmp_nodes.size() > 1 && !mWorld->collisionFree(tmp_nodes[i], tmp_nodes[i - 1])))
                return false;
            // if the distance between the current node and the new root is less
            // than the threshold and there is no obstacle between them.
            if (EuclideanDistance(tmp_nodes[i], new_root) < mInheritThreshold &&
                mWorld->collisionFree(tmp_nodes[i], new_root))
            {
                result = true;
                break;
            }
        }
        if (result)
        {
            tmp_nodes.push_back(new_root);
            size_t start_index = (type == Path::Global ? 1 : 0);
            for (size_t i = start_index; i < tmp_nodes.size() - 1; i++)
            {
                tmp_nodes[i]->parent = tmp_nodes[i + 1];
                tmp_nodes[i + 1]->children.push_back(tmp_nodes[i]);
            }
            mPath = Path();
            mTree.clear();
            mTree.assign(tmp_nodes.begin() + start_index, tmp_nodes.end());
            mOriginNode = new_root;
            updateNode(mOriginNode);
            generatePath();
        }
    }
    return result;
}

float PFRRTStar::getRandomNum()
{
    std::random_device rand_rd;
    std::mt19937 rand_gen(rand_rd());
    std::uniform_real_distribution<> rand_unif(0, 1.0);
    return rand_unif(rand_gen);
}

Eigen::Vector2d PFRRTStar::getRandom2DPoint()
{
    Eigen::Vector3d lb = mWorld->getLowerBound();
    Eigen::Vector3d ub = mWorld->getUpperBound();

    Eigen::Vector2d rand_point =
        Eigen::Vector2d((ub(0) - lb(0)) * getRandomNum() + lb(0), (ub(1) - lb(1)) * getRandomNum() + lb(1));

    return rand_point;
}

Eigen::Vector3d PFRRTStar::sampleInEllipsoid()
{
    float cmin = EuclideanDistance(mTargetNode, mOriginNode);
    Eigen::Vector3d a_1 = (mTargetNode->position - mOriginNode->position) / cmin;
    Eigen::RowVector3d id_t(1, 0, 0);
    Eigen::Matrix3d M = a_1 * id_t;
    Eigen::JacobiSVD<Eigen::MatrixXd> SVD(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = SVD.matrixU();
    Eigen::Matrix3d V = SVD.matrixV();
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    A(0, 0) = A(1, 1) = 1, A(2, 2) = U.determinant() * V.determinant();
    Eigen::Matrix3d C = U * A * V;

    float cbest = mPath.distance + 1.0f;

    Eigen::Matrix3d L = Eigen::Matrix3d::Zero();
    L(0, 0) = cbest * 0.5, L(1, 1) = L(2, 2) = sqrt(powf(cbest, 2) - powf(cmin, 2)) * 0.5;

    float theta1 = acos(2 * getRandomNum() - 1);
    float theta2 = 2 * M_PI * getRandomNum();
    float radius = powf(getRandomNum(), 1.0 / 3);

    Eigen::Vector3d random_ball;
    random_ball << radius * sin(theta1) * cos(theta2), radius * sin(theta1) * sin(theta2), radius * cos(theta1);

    Eigen::Vector3d random_ellipsoid = C * L * random_ball;

    Eigen::Vector3d center = (mOriginNode->position + mTargetNode->position) * 0.5;
    Eigen::Vector3d point = random_ellipsoid + center;
    return point;
}

Eigen::Vector2d PFRRTStar::sampleInSector()
{
    float sample_sector_lb_ = 2.0f;

    std::vector<float> theta_record;

    Eigen::Vector2d start_2D = project2plane(mOriginNode->position);
    for (size_t i = 0; i < mPath.nodes.size() - 1; i++)
    {
        Eigen::Vector2d pt_2D = project2plane(mPath.nodes[i]->position);
        Eigen::Vector2d diff = pt_2D - start_2D;
        if (diff.norm() < sample_sector_lb_) continue;
        float theta = atan2f(diff(1), diff(0));
        theta_record.push_back(theta);
    }

    if (theta_record.empty()) return getRandom2DPoint();

    std::default_random_engine engine;

    std::uniform_int_distribution<unsigned> rand_int(0, theta_record.size() - 1);

    float theta_rate = getRandomNum();

    float theta = theta_record[rand_int(engine)] + (theta_rate - 0.5) * 20.0 * M_PI / 180.0;

    Eigen::Vector2d sub_goal_2D = project2plane(mPath.nodes.front()->position);

    float sample_sector_ub =
        (EuclideanDistance(start_2D, sub_goal_2D) + EuclideanDistance(start_2D, mEndPos_2d)) * 0.5 + 1.0f;

    float rand_num = getRandomNum();

    float R = sqrt(rand_num) * (sample_sector_ub - sample_sector_lb_) + sample_sector_lb_;

    Eigen::Vector2d rand_point(R * cos(theta), R * sin(theta));
    rand_point += project2plane(mOriginNode->position);

    return rand_point;
}

Eigen::Vector2d PFRRTStar::sample()
{
    Eigen::Vector2d point_sample;
    switch (mPlanningState)
    {
        case Global:
            if (!mPath.nodes.empty())
                point_sample = project2plane(sampleInEllipsoid());
            else
                point_sample =
                    (getRandomNum() < mGoalBiased) ? project2plane(mTargetNode->position) : getRandom2DPoint();
            break;
        case Roll:
            point_sample = mPath.nodes.empty() ? getRandom2DPoint() : sampleInSector();
            break;
        case WithoutGoal:
            point_sample = getRandom2DPoint();
            break;
        default:
            break;
    }
    return point_sample;
}

Node::Ptr PFRRTStar::findNearest(const Eigen::Vector2d& point)
{
    float min_dis = gINF;
    Node::Ptr node_closest = nullptr;

    for (const auto& node : mTree)
    {
        // Here use Manhattan distance instead of Euclidean distance to improve the calculate speed.
        float tmp_dis = std::fabs(point(0) - node->position(0)) + std::fabs(point(1) - node->position(1));
        if (tmp_dis < min_dis)
        {
            min_dis = tmp_dis;
            node_closest = node;
        }
    }
    return node_closest;
}

Eigen::Vector2d PFRRTStar::steer(const Eigen::Vector2d& point_rand_projection,
                                 const Eigen::Vector2d& point_nearest_projection)
{
    Eigen::Vector2d point_new_projection;
    Eigen::Vector2d steer_p = point_rand_projection - point_nearest_projection;
    float steer_norm = steer_p.norm();

    // check if the EuclideanDistance between two nodes is larger than the maximum travel step size
    if (steer_norm > mStepSize)
        point_new_projection = point_nearest_projection + steer_p * mStepSize / steer_norm;
    else
        point_new_projection = point_rand_projection;
    return point_new_projection;
}

Node::Ptr PFRRTStar::fitPlane(const Eigen::Vector2d& p_original)
{
    Node::Ptr node = nullptr;
    Eigen::Vector3d p_surface;

    /* Make sure that p_original can be projected to the surface,otherwise the nullptr will be returned */
    if (mWorld->project2surface(p_original(0), p_original(1), p_surface))
    {
        node = std::make_shared<Node>();
        node->plane = std::make_shared<Plane>(p_surface, mWorld, mRadiusFitPlane, mFitPlaneArg);
        node->position = p_surface + mHfromSurf * node->plane->normal_vector;
    }
    return node;
}

void PFRRTStar::fitPlane(Node::Ptr node)
{
    Eigen::Vector2d init_coord = node->plane->init_coord;
    node->plane = nullptr;
    Eigen::Vector3d p_surface;
    if (mWorld->project2surface(init_coord(0), init_coord(1), p_surface))
    {
        node->plane = std::make_shared<Plane>(p_surface, mWorld, mRadiusFitPlane, mFitPlaneArg);
        node->position = p_surface + mHfromSurf * node->plane->normal_vector;
    }
}

void PFRRTStar::findNearNeighbors(Node::Ptr node_new, std::vector<std::pair<Node::Ptr, float>>& record)
{
    for (const auto& node : mTree)
    {
        if (EuclideanDistance(node_new, node) < mNeighborRadius && mWorld->collisionFree(node_new, node))
            record.push_back(std::pair<Node::Ptr, float>(node, CostBetweenTwoNode(node_new, node)));
    }
}

void PFRRTStar::findParent(Node::Ptr node_new, const std::vector<std::pair<Node::Ptr, float>>& record)
{
    Node::Ptr parent_node = nullptr;
    float min_cost = gINF;
    for (const auto& rec : record)
    {
        Node::Ptr node = rec.first;
        float tmp_cost = node->cost + rec.second;
        if (tmp_cost < min_cost)
        {
            parent_node = node;
            min_cost = tmp_cost;
        }
    }
    node_new->parent = parent_node;
    node_new->cost = min_cost;
    parent_node->children.push_back(node_new);
}

void PFRRTStar::reWire(Node::Ptr node_new, const std::vector<std::pair<Node::Ptr, float>>& record)
{
    for (const auto& rec : record)
    {
        Node::Ptr node = rec.first;
        float tmp_cost = node_new->cost + rec.second;   // cost value if the new node is the parent
        float cost_difference =
            node->cost -
            tmp_cost;   // compare the two and update if the latter is smaller,change new node to the parent node
        if (cost_difference > 0)
        {
            deleteChildren(node->parent, node);
            node->parent = node_new;
            node->cost = tmp_cost;
            node_new->children.push_back(node);
            updateChildrenCost(node, cost_difference);
        }
    }
}

void PFRRTStar::deleteChildren(Node::Ptr parent_node, Node::Ptr child_node)
{
    for (std::vector<Node::Ptr>::iterator it = parent_node->children.begin(); it != parent_node->children.end(); ++it)
    {
        if (*it == child_node)
        {
            parent_node->children.erase(it);
            break;
        }
    }
}

void PFRRTStar::updateChildrenCost(Node::Ptr& node_root, float cost_difference)
{
    for (auto& node : node_root->children)
    {
        node->cost -= cost_difference;
        updateChildrenCost(node, cost_difference);
    }
}

void PFRRTStar::closeCheck(Node::Ptr node)
{
    switch (mPlanningState)
    {
        case Global:
            if (EuclideanDistance(node, mTargetNode) < mGoalThreshold && mWorld->collisionFree(node, mTargetNode))
            {
                mCloseCheckRecord.push_back(std::pair<Node::Ptr, float>(node, CostBetweenTwoNode(node, mTargetNode)));
            }
            break;
        case Roll:
            if (EuclideanDistance(project2plane(node->position), mEndPos_2d) < mSubGoalThreshold)
            {
                mCloseCheckRecord.push_back(std::pair<Node::Ptr, float>(
                    node, powf(EuclideanDistance(mEndPos_2d, project2plane(node->position)), 3)));
            }
            break;
        default:
            break;
    }
}

float PFRRTStar::PathDistance(const std::vector<Node::Ptr>& nodes)
{
    float distance = 0.0f;
    for (size_t i = 0; i < nodes.size() - 1; i++)
    {
        distance += EuclideanDistance(nodes[i], nodes[i + 1]);
    }
    return distance;
}

void PFRRTStar::generatePath()
{
    if (mPlanningState == Global)
    {
        Node::Ptr node_choosed = nullptr;
        float min_cost = mPath.cost;
        for (const auto& rec : mCloseCheckRecord)
        {
            Node::Ptr node = rec.first;
            float tmp_cost = node->cost + rec.second;
            if (tmp_cost < min_cost)
            {
                min_cost = tmp_cost;
                node_choosed = node;
            }
        }
        if (min_cost != mPath.cost)
        {
            mPath.nodes.clear();
            mPath.nodes.push_back(mTargetNode);
            while (node_choosed != nullptr)
            {
                mPath.nodes.push_back(node_choosed);
                node_choosed = node_choosed->parent;
            }
            mPath.cost = min_cost;
            mPath.distance = PathDistance(mPath.nodes);
            mPath.type = Path::Global;
        }
    }
    else if (mPlanningState == Roll)
    {
        mPath = Path();
        Node::Ptr sub_goal = nullptr;
        float min_cost = gINF;
        for (const auto& rec : mCloseCheckRecord)
        {
            Node::Ptr node = rec.first;
            float tmp_cost = node->cost + rec.second;
            if (tmp_cost < min_cost)
            {
                min_cost = tmp_cost;
                sub_goal = node;
            }
        }
        if (sub_goal != nullptr)
        {
            while (sub_goal != nullptr)
            {
                mPath.nodes.push_back(sub_goal);
                sub_goal = sub_goal->parent;
            }
            mPath.cost = mPath.nodes.front()->cost;
            mPath.distance = PathDistance(mPath.nodes);
            mPath.type = Path::Sub;
        }
        else
        {
            // If close_check_record is empty,adjust the threshold according to the current information of distance
            // so that next time it will more likely to get a solution
            float min_dis = gINF;
            for (const auto& node : mTree)
            {
                float tmp_dis = EuclideanDistance(project2plane(node->position), mEndPos_2d);
                if (tmp_dis < min_dis) min_dis = tmp_dis;
            }
            mSubGoalThreshold = min_dis + 1.0f;
        }
    }
}

Path PFRRTStar::planner(int max_iter, double max_time)
{
    if (mPlanningState == Invalid)
    {
        RCLCPP_ERROR(mRclNode->get_logger(), "Illegal operation: the planner is at an invalid working state!!");
        return {};
    }
    double time_now = mCurrTime;
    timeval start;
    gettimeofday(&start, nullptr);
    while (mCurrIter < max_iter && mCurrTime < max_time)
    {

        mCurrIter++;   // Update current iteration

        // Update current time consuming
        timeval end;
        gettimeofday(&end, nullptr);
        float ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
        mCurrTime = ms + time_now;


        Eigen::Vector2d rand_point_2D = sample();   // Sample to get a random 2D point

        /* Find the nearest node to the random point */
        Node::Ptr nearest_node = findNearest(rand_point_2D);
        Eigen::Vector2d nearest_point_2D = project2plane(nearest_node->position);

        /* Expand from the nearest point to the random point */
        Eigen::Vector2d new_point_2D = steer(rand_point_2D, nearest_point_2D);


        Node::Ptr new_node = fitPlane(new_point_2D);   // Based on the new 2D point

        if (new_node != nullptr                              // 1.Fail to fit the plane, it will return a null pointer
            && mWorld->isInsideBorder(new_node->position))   // 2.The position is out of the range of the grid map.
        {
            /* Get the set of the neighbors of the new node in the tree */
            std::vector<std::pair<Node::Ptr, float>> neighbor_record;
            findNearNeighbors(new_node, neighbor_record);

            // Select an appropriate parent node for the new node from the set.
            if (!neighbor_record.empty()) findParent(new_node, neighbor_record);
            // Different from other RRT algorithm, it is posible that the new node is too far away from the whole tree.
            // If so, discard the new node.
            else
            {
                continue;
            }
            mTree.push_back(new_node);           // Add the new node to the tree
            reWire(new_node, neighbor_record);   // Rewire the tree to optimize it
            closeCheck(new_node);                // Check if the new node is close enough to the goal

            if (mPlanningState == Global)
            {
                generatePath();
            }
        }
    }

    if (mPlanningState == Roll) generatePath();

    // visualizeTree(mTree, tree_vis_pub_);
    mRclNode->visualizeTree(mTree);
    pubTraversabilityOfTree();

    return mPath;
}

void PFRRTStar::pubTraversabilityOfTree()
{
    if (mRclNode->mTreeTraPub == nullptr) return;
    std_msgs::msg::Float32MultiArray msg;
    for (const auto& node : mTree)
    {
        msg.data.push_back(node->position(0));
        msg.data.push_back(node->position(1));
        msg.data.push_back(node->position(2));
        msg.data.push_back(node->plane->traversability);
    }
    mRclNode->mTreeTraPub->publish(msg);
}
}   // namespace planner
}   // namespace putn
