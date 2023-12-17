#pragma once
// #include <utility>
#include "DataTypes.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
// #include "Visualization.h"

namespace putn
{
namespace planner
{
/**
 * @brief Represents four different working states of the planner.
 */
enum PlanningState
{
    Global,
    Roll,
    WithoutGoal,
    Invalid
};

class PFRRTStar
{
public:
    explicit PFRRTStar() = default;

    /* Input the height of the robot center,and the array of grid map. */
    explicit PFRRTStar(double height, World::Ptr world) : mH_surf(height), mWorld(world)
    {
    }
    ~PFRRTStar() = default;

    /**
     * @brief Set the origin and target for the planner.According to whether they are successfully projected to the
     *        surface,the planner will convert to 3 different working states: Global, Roll, Invalid.
     * @param start_point
     * @param end_point
     * @note In fact, only the x and y dimensions of the input point are used
     */
    void initWithGoal(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos);

    /**
     * @brief Only set the origin for the planner.According to whether it's successfully projected to the
     *        surface,the planner will convert to 2 different working states:WithoutGoal,Invalid.
     * @param start_point
     */
    void initWithoutGoal(const Eigen::Vector3d& start_pos);

    /**
     * @brief Expand the tree to search the solution,and stop after reaching the max iterations or max time.
     * @param max_iter
     * @param max_time
     * @return Path(The solution to the goal)
     */
    Path planner(int max_iter, double max_time);

    int getCurrentIterations()
    {
        return mCurrIter;
    }

    double getCurrentTime()
    {
        return mCurrTime;
    }

    void setFitPlaneRadius(float radius)
    {
        mRadiusFitPlane = radius;
    }

    void setFitPlaneArg(const FitPlaneArg& fit_plane_arg)
    {
        mFitPlaneArg = fit_plane_arg;
    }

    void setStepSize(double step_size)
    {
        mStepSize = step_size;
    }

    void setGoalThre(double threshold)
    {
        mGoalThreshold = threshold;
    }

    void setGoalBiased(double goal_biased)
    {
        mGoalBiased = goal_biased;
    }

    void setNeighborRadius(double neighbor_radius)
    {
        mNeighborRadius = neighbor_radius;
    }

    Node::Ptr origin()
    {
        return mOriginNode;
    }

    Node::Ptr target()
    {
        return mTargetNode;
    }

    std::vector<Node::Ptr> tree()
    {
        return mTree;
    }

    Path path()
    {
        return mPath;
    }

    PlanningState state()
    {
        return mPlanningState;
    }

    /**
     * @brief Project a point to the surface and then fit a local plane on it.Generate a new node based on the plane.
     * @param p_original
     * @return Node*
     */
    Node::Ptr fitPlane(const Eigen::Vector2d& p_original);
    Node::Ptr fitPlane(const Eigen::Vector3d& p_original)
    {
        return fitPlane(project2plane(p_original));
    }

    /**
     * @brief According to the init coordinates stored by the node,update its information about plane and position.
     * @param node
     * @note Unlike the above function,it doesn't create new nodes,but updates the existing node
     */
    void fitPlane(Node::Ptr node);

protected:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mTreeTraPub{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mGoalVisualizePub{nullptr},
        mTreeVisualizePub{nullptr};

    Node::Ptr mOriginNode, mTargetNode;

    int mCurrIter;
    double mCurrTime;   // Unit: ms

    // To accelerate the speed of generating the initial solution,the tree will grow toward the target with it,a centain
    // probability
    double mGoalBiased{0.15}, mGoalThreshold{1.0}, mSubGoalThreshold{1.0}, mInheritThreshold{1.25};

    double mStepSize = 0.2;   // step size used when generating new nodes

    /* Parameters related to function fitPlane */
    float mH_surf;
    FitPlaneArg mFitPlaneArg = {1.0, 2000.0, 0.0014, 0.4, 0.25, 0.4, 0.1152};
    double mRadiusFitPlane = 1.0;

    float mNeighborRadius = 1.0f;   // radius used in function FindNeighbors

    PlanningState mPlanningState;

    World::Ptr mWorld;

    std::vector<std::pair<Node::Ptr, float>> mCloseCheckRecord;   // used in function generatePath

    std::vector<Node::Ptr> mTree;

    Path mPath;

    /* record 2D information of target, when the target can't be projected to surface,it will be used in
     rolling-planning*/
    Eigen::Vector2d mEndPos_2d;


    /*------ Functions for inherit ------*/
    void updateNode(Node::Ptr input_node);

    bool inheritPath(Node::Ptr new_root, Path::Type type);

    void addInvalidNodes(Node::Ptr& input_node, bool ifdelete, std::vector<Node::Ptr>& invalid_nodes);

    void trimTree();

    bool inheritTree(Node::Ptr new_root);
    /*---------------------------------*/

    /*------ Functions for sample ------*/
    float getRandomNum();

    Eigen::Vector2d getRandom2DPoint();

    Eigen::Vector3d sampleInEllipsoid();

    Eigen::Vector2d sampleInSector();

    /**
     * @brief Sample to get a random 2D point in various ways.It integrates all the above sampling functions
     * @return Vector2d
     */
    Eigen::Vector2d sample();
    /*---------------------------------*/

    Node::Ptr findNearest(const Eigen::Vector2d& point);

    Eigen::Vector2d steer(const Eigen::Vector2d& point_rand_projection, const Eigen::Vector2d& point_nearest);

    void findNearNeighbors(Node::Ptr new_node, std::vector<std::pair<Node::Ptr, float>>& record);

    void findParent(Node::Ptr new_node, const std::vector<std::pair<Node::Ptr, float>>& record);

    void reWire(Node::Ptr new_node, const std::vector<std::pair<Node::Ptr, float>>& record);

    void deleteChildren(Node::Ptr parent_node, Node::Ptr child_node);

    void updateChildrenCost(Node::Ptr& node_root, float cost_difference);

    /**
     * @brief Check the node. If it has met the conditions set in advance(i.e.,it's close enough to the target),
     *        add it to the data member `mCloseCheckRecord`.
     * @param node
     */
    void closeCheck(Node::Ptr node);

    /**
     * @brief Read information from "mCloseCheckRecord".According to the node information stored in,the function
     *        will select the node with the smallest valuation funtion,and generate a path through the node
     */
    void generatePath();

    float calPathDis(const std::vector<Node::Ptr>& nodes);

    void pubTraversabilityOfTree();
};
}   // namespace planner
}   // namespace putn