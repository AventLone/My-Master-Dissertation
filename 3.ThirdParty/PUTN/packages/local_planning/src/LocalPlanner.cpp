#include "LocalPlanner.h"

LocalPlanner::LocalPlanner(const std::string& name) : rclcpp::Node(name)
{
    int replan_interval = 10;   // Unit: ms
    declare_parameter("LocalPlanner.ReplanInterval", 10);
    get_parameter("LocalPlanner.ReplanInterval", replan_interval);

    /* Initiate Subscriptions */
    mCurrStateSub = create_subscription<std_msgs::msg::Float32MultiArray>(
        "current_state",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&LocalPlanner::currStateCallback, this, std::placeholders::_1));
    mObstacleSub = create_subscription<std_msgs::msg::Float32MultiArray>(
        "obstacle",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&LocalPlanner::obstacleCallback, this, std::placeholders::_1));
    mGoalPathSub = create_subscription<std_msgs::msg::Float32MultiArray>(
        // "surf_predict_pub",
        "putn/gpr/surface_prediction",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&LocalPlanner::globalPathCallback, this, std::placeholders::_1));

    /* Initiate Publishers */
    mObstaclePub = create_publisher<visualization_msgs::msg::MarkerArray>("draw_obstacles",
                                                                          rclcpp::SystemDefaultsQoS().reliable());
    mLocalPathPub = create_publisher<nav_msgs::msg::Path>("putn/local_path", rclcpp::ParametersQoS().reliable());
    mLocalPlanPub =
        create_publisher<std_msgs::msg::Float32MultiArray>("putn/local_plan", rclcpp::ParametersQoS().reliable());

    /* Initiate Timer */
    mTimer =
        create_wall_timer(std::chrono::milliseconds(replan_interval), std::bind(&LocalPlanner::replanCallback, this));
}

void LocalPlanner::drawObstacle()
{
    mObstacleMarkArray.markers.clear();
    for (std::size_t i = 0; i < mObstacle.size(); ++i)
    {
        visualization_msgs::msg::Marker temp_obstacle;
        temp_obstacle.header.frame_id = "world";
        temp_obstacle.id = i;
        temp_obstacle.type = temp_obstacle.CYLINDER;
        temp_obstacle.action = temp_obstacle.ADD;
        temp_obstacle.pose.position.x = mObstacle[i][0];
        temp_obstacle.pose.position.y = mObstacle[i][1];
        temp_obstacle.pose.position.z = 0.2;
        temp_obstacle.scale.x = 0.1;
        temp_obstacle.scale.y = 0.1;
        temp_obstacle.scale.z = 0.4;
        temp_obstacle.color.a = 1;
        temp_obstacle.color.r = 0;
        temp_obstacle.color.g = 1;
        temp_obstacle.color.b = 0;
        mObstacleMarkArray.markers.push_back(temp_obstacle);
    }
    mObstaclePub->publish(mObstacleMarkArray);
}

/**
 * @brief Return the index of minimum element in a std::vector.
 */
template<typename T>
inline long argmin(const std::vector<T>& arr)
{
    auto min_value = std::min_element(arr.begin(), arr.end());
    return std::distance(arr.begin(), min_value);
}

void LocalPlanner::chooseGoalState()
{
    std::vector<double> distances;
    for (int i = 0; i < mDesiredGlobalPath.second; ++i)
    {
        // distances.push_back(globalDistance(mCurrState, mDesiredGlobalPath.first(i, Eigen::all)));
        auto globla_distance = std::sqrt((mCurrState[0] - mDesiredGlobalPath.first(i, Eigen::all)[0]) *
                                             (mCurrState[0] - mDesiredGlobalPath.first(i, Eigen::all)[0]) +
                                         (mCurrState[1] - mDesiredGlobalPath.first(i, Eigen::all)[1]) *
                                             (mCurrState[1] - mDesiredGlobalPath.first(i, Eigen::all)[1]));
        distances.push_back(globla_distance);
    }

    /* Get the index of the minimum element in a STL container. */
    long num = std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));

    int scale = 1;
    std::vector<int> num_list;

    for (int i; i < N; ++i)
    {
        auto num_path = std::min(mDesiredGlobalPath.second - 1, static_cast<int>(num + i * scale));
        num_list.push_back(num_path);
    }
    if (num >= mDesiredGlobalPath.second)
    {
        mIsEnd = true;
    }
    for (int i; i < N; ++i)
    {
        mGoalState(i, Eigen::all) = mDesiredGlobalPath.first(num_list[i], Eigen::all);
    }
}

void LocalPlanner::pubLocalPlan(const Eigen::MatrixXd& input_solution, const Eigen::MatrixXd& state_solution)
{
    nav_msgs::msg::Path local_path;
    std_msgs::msg::Float32MultiArray local_plan;
    local_path.header.stamp = this->now();
    local_path.header.frame_id = "world";
    for (int i = 0; i < N; ++i)
    {
        geometry_msgs::msg::PoseStamped temp_pose;
        temp_pose.pose.position.x = state_solution(i, 0);
        temp_pose.pose.position.y = state_solution(i, 1);
        temp_pose.pose.position.z = z + 0.5;
        temp_pose.header.stamp = this->now();
        temp_pose.header.frame_id = "world";
        local_path.poses.push_back(temp_pose);
        for (int j = 0; j < 2; ++j)
        {
            local_plan.data.push_back(input_solution(i, j));
        }
    }
    mLocalPathPub->publish(local_path);
    mLocalPlanPub->publish(local_plan);
}

void LocalPlanner::currStateCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg)
{
    mRobotStateSet = true;
    mCurrState(0) = msg->data[0];
    mCurrState(1) = msg->data[1];
    mCurrState(2) = msg->data[3];
    mCurrState(3) = msg->data[4];
    mCurrState(4) = msg->data[5];

    z = msg->data[2];
}

void LocalPlanner::globalPathCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg)
{
    if (msg->data.empty()) return;
    ref_path_close_set = true;
    auto size = msg->data.size() / 5;
    mDesiredGlobalPath.second = size;
    for (std::size_t i = 0; i < size; ++i)
    {
        mDesiredGlobalPath.first(i, 0) = msg->data[5 * (size - i) - 5];
        mDesiredGlobalPath.first(i, 1) = msg->data[5 * (size - i) - 4];
        mDesiredGlobalPath.first(i, 2) = msg->data[5 * (size - i) - 2];
        mDesiredGlobalPath.first(i, 3) = msg->data[5 * (size - i) - 1];
    }
}

void LocalPlanner::obstacleCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg)
{
    if (msg->data.empty()) return;

    auto size = msg->data.size() / 3;
    for (std::size_t i = 0; i < size; ++i)
    {
        Eigen::Vector2d temp_point(std::floor(msg->data[3 * i] / 0.3) * 0.3,
                                   std::floor(msg->data[3 * i + 1] / 0.3) * 0.3);
        mObstacle.emplace_back(temp_point);
    }
    drawObstacle();
}

void LocalPlanner::replanCallback()
{
    if (mRobotStateSet && mRefPathSet)
    {
        chooseGoalState();
        mMPC.buildProblem(mCurrState, mGoalState, mObstacle);
        if (!mIsEnd)
        {
            pubLocalPlan(mMPC.getInputSolution(), mMPC.getStateSolution());
        }
        mHavePlan = true;
    }
    else if (!mRobotStateSet && mRefPathSet)
    {
        RCLCPP_WARN(get_logger(), "No pose.");
    }
    else if (mRobotStateSet && !mRefPathSet)
    {
        RCLCPP_WARN(get_logger(), "No path.");
    }
    else
    {
        RCLCPP_WARN(get_logger(), "No pose and no path.");
    }
}
