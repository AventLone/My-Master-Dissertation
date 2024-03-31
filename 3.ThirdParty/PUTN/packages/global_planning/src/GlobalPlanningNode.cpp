#include "GlobalPlanningNode.h"

GlobalPlanningNode::GlobalPlanningNode(const std::string& name) : rclcpp::Node(name)
{
    declare_parameter("Map.Resolution", 0.1);
    declare_parameter("Plannig.GoalThreshold", 1.0);
    declare_parameter("Plannig.StepSize", 0.2);
    declare_parameter("Plannig.CartHeight", 0.36);   // The height from surface to cart bottom, default: 0.4
    declare_parameter("Plannig.NeighborRadius", 1.0);
    declare_parameter("Plannig.FitPlane", 0.4);
    declare_parameter("Plannig.Flatness", 4000.0);
    declare_parameter("Plannig.Slope", 0.4);
    declare_parameter("Plannig.Sparsity", 0.4);
    declare_parameter("Plannig.RatioMin", 0.25);
    declare_parameter("Plannig.RatioMax", 0.4);
    declare_parameter("Plannig.ConvThre", 0.1152);
    declare_parameter("Plannig.FitPlaneRadius", 0.8);   // default: 1.0
    declare_parameter("Plannig.MaxInitTime", 1000.0);

    double resolution, step_size;
    double h_surf_car, fit_plane_radius, neighbor_radius;
    putn::FitPlaneArg fit_plane_arg;
    get_parameter("Map.Resolution", resolution);
    get_parameter("Plannig.GoalThreshold", mGoalThreshold);
    get_parameter("Plannig.StepSize", step_size);
    get_parameter("Plannig.CartHeight", h_surf_car);   // The height from surface to cart bottom
    get_parameter("Plannig.NeighborRadius", neighbor_radius);
    get_parameter("Plannig.FitPlane", fit_plane_arg.w_total);
    get_parameter("Plannig.Flatness", fit_plane_arg.w_flatness);
    get_parameter("Plannig.Slope", fit_plane_arg.w_slope);
    get_parameter("Plannig.Sparsity", fit_plane_arg.w_sparsity);
    get_parameter("Plannig.RatioMin", fit_plane_arg.ratio_min);
    get_parameter("Plannig.RatioMax", fit_plane_arg.ratio_max);
    get_parameter("Plannig.ConvThre", fit_plane_arg.conv_thre);
    get_parameter("Plannig.FitPlaneRadius", fit_plane_radius);
    get_parameter("Plannig.MaxInitTime", mMaxInitTime);

    mWorld = std::make_shared<putn::World>(resolution);
    mPFRRTStar = std::make_unique<putn::planner::PFRRTStar>(h_surf_car, mWorld, this);
    mPFRRTStar->setGoalThre(mGoalThreshold);
    mPFRRTStar->setStepSize(step_size);
    mPFRRTStar->setFitPlaneArg(fit_plane_arg);
    mPFRRTStar->setFitPlaneRadius(fit_plane_radius);
    mPFRRTStar->setNeighborRadius(neighbor_radius);

    mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

    mTimer = create_wall_timer(std::chrono::milliseconds(80), std::bind(&GlobalPlanningNode::callPlanner, this));

    /* Initiate Subscription */
    mMapSub = create_subscription<sensor_msgs::msg::PointCloud2>(
        "orb_slam3/cloud_map",
        rclcpp::SensorDataQoS().best_effort(),
        std::bind(&GlobalPlanningNode::cloudMapCallback, this, std::placeholders::_1));
    mGoalSub = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&GlobalPlanningNode::goalCallback, this, std::placeholders::_1));
    // mGoalSub = create_subscription<nav_msgs::msg::Path>(
    //     "putn/waypoints",
    //     rclcpp::SensorDataQoS().reliable(),
    //     std::bind(&GlobalPlanningNode::goalCallback, this, std::placeholders::_1));


    /* Initiate Publisher */
    mGridMapVisualizePub = create_publisher<octomap_msgs::msg::Octomap>("putn/global_planning/grid_map_vis",
                                                                        rclcpp::SensorDataQoS().best_effort());
    mPathVisualizePub = create_publisher<visualization_msgs::msg::Marker>("putn/global_planning/path_vis",
                                                                          rclcpp::SensorDataQoS().best_effort());
    mGoalVisualizePub = create_publisher<visualization_msgs::msg::Marker>("putn/global_planning/goal_vis",
                                                                          rclcpp::SensorDataQoS().best_effort());
    mPlaneVisualizePub = create_publisher<sensor_msgs::msg::PointCloud2>("putn/global_planning/surf_vis",
                                                                         rclcpp::SensorDataQoS().best_effort());
    mTreeVisualizePub = create_publisher<visualization_msgs::msg::Marker>("putn/global_planning/tree_vis",
                                                                          rclcpp::SensorDataQoS().best_effort());

    mTreeTraPub = create_publisher<std_msgs::msg::Float32MultiArray>("putn/global_planning/tree_tra",
                                                                     rclcpp::ServicesQoS().reliable());
    mGlobalPathPub = create_publisher<std_msgs::msg::Float32MultiArray>("putn/global_planning/global_path",
                                                                        rclcpp::ServicesQoS().reliable());
}

// void GlobalPlanningNode::goalCallback(const nav_msgs::msg::Path::ConstSharedPtr& msg)
void GlobalPlanningNode::goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
    if (!mWorld->exist_map) return;
    mHasGoal = true;
    mTargetPoint = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, 0.1);
    RCLCPP_INFO(get_logger(), "Receive the planning target.");
}

void GlobalPlanningNode::cloudMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    mWorld->initGridMap(cloud);

    for (const auto& pt : cloud)
    {
        Eigen::Vector3d obstacle(pt.x, pt.y, pt.z);
        mWorld->setObstacle(obstacle);
    }
    visualizeWorld();
}

void GlobalPlanningNode::pubGlobalPath(const std::vector<putn::Node::Ptr>& solution)
{
    std_msgs::msg::Float32MultiArray msg;
    for (size_t i = 0; i < solution.size(); ++i)
    {
        if (i == solution.size() - 1)
        {
            msg.data.push_back(solution[i]->position(0));
            msg.data.push_back(solution[i]->position(1));
            msg.data.push_back(solution[i]->position(2));
        }
        else
        {
            size_t interpolation_num = static_cast<size_t>(putn::EuclideanDistance(solution[i + 1], solution[i]) / 0.1);
            Eigen::Vector3d diff_pt = solution[i + 1]->position - solution[i]->position;
            for (size_t j = 0; j < interpolation_num; ++j)
            {
                Eigen::Vector3d interpt = solution[i]->position + diff_pt * (float)j / interpolation_num;
                msg.data.push_back(interpt(0));
                msg.data.push_back(interpt(1));
                msg.data.push_back(interpt(2));
            }
        }
    }
    mGlobalPathPub->publish(msg);
}

void GlobalPlanningNode::findSolution()
{
    putn::Path solution = putn::Path();

    mPFRRTStar->initWithGoal(mStartPoint, mTargetPoint);

    /* Case 1: The PF-RRT* can't work at when the origin can't be projected to surface. */
    if (mPFRRTStar->state() == putn::planner::Invalid)
    {
        RCLCPP_WARN_ONCE(get_logger(), "The start point can't be projected. Unable to start PF-RRT* algorithm!!!");
    }

    /* Case 2: If both the origin and the target can be projected,the PF-RRT* will
       execute global planning and try to generate a path. */
    else if (mPFRRTStar->state() == putn::Path::Global)
    {
        int max_iter = 5000;
        double max_time = 100.0;

        while (solution.type == putn::Path::Empty && max_time < mMaxInitTime)
        {
            solution = mPFRRTStar->planner(max_iter, max_time);
            max_time += 100.0;
        }

        if (!solution.nodes.empty())
        {
            RCLCPP_INFO(get_logger(), "Get a global path.");
        }
        else
        {
            RCLCPP_WARN(get_logger(), "No solution found!");
        }
    }

    /* Case 3: If the origin can be projected while the target can not,
       the PF-RRT* will try to find a temporary target for transitions. */
    else
    {
        int max_iter = 1500;
        double max_time = 100.0;

        solution = mPFRRTStar->planner(max_iter, max_time);

        if (!solution.nodes.empty())
        {
            RCLCPP_INFO(get_logger(), "Get a sub path!");
        }
        else
        {
            RCLCPP_WARN(get_logger(), "No solution found!");
        }
    }

    pubGlobalPath(solution.nodes);
    visualizePath(solution.nodes);
    visualizePlane(solution.nodes);

    /* When the PF-RRT* generates a short enough global path,
       it's considered that the robot has reached the goal region. */
    if (solution.type == putn::Path::Global &&
        putn::EuclideanDistance(mPFRRTStar->origin(), mPFRRTStar->target()) < mGoalThreshold)
    {
        mHasGoal = false;
        visualizeOriginAndGoal({});   // Passing an empty set to delete the previous display
        visualizePath({});
        RCLCPP_INFO(get_logger(), "The robot has achieved the goal!!");
    }

    if (solution.type == putn::Path::Empty)
    {
        visualizePath({});
    }
}

void GlobalPlanningNode::callPlanner()
{
    //---------
    geometry_msgs::msg::TransformStamped transform_msg;
    try
    {
        transform_msg = mTfBuffer->lookupTransform("world", "base_link", tf2::TimePointZero);
    }
    catch (const tf2::TransformException& e)
    {
        return;
    }
    // Eigen::Vector3d start_point;
    // start_point << transform_msg.transform.translation.x, transform_msg.transform.translation.y,
    //     transform_msg.transform.translation.z;
    mStartPoint << transform_msg.transform.translation.x, transform_msg.transform.translation.y, 0.1;

    RCLCPP_INFO_STREAM_ONCE(get_logger(), "Transform from 'world' to 'base_link' is:" << mStartPoint);
    //--------

    static double init_time_cost = 0.0;
    if (!mWorld->exist_map) return;

    /* The tree will expand at a certain frequency to explore the space more fully. */
    if (!mHasGoal && init_time_cost < 1000.0)
    {
        mPFRRTStar->initWithoutGoal(mStartPoint);

        if (mPFRRTStar->state() == putn::planner::WithoutGoal)
        {
            int max_iter = 550;
            double max_time = 100.0;
            mPFRRTStar->planner(max_iter, max_time);
            // RCLCPP_INFO(get_logger(), "Current size of tree: %d", static_cast<int>(mPFRRTStar->tree().size()));
        }
        else
        {
            RCLCPP_WARN_ONCE(get_logger(), "The start point can't be projected, unable to execute PF-RRT* algorithm.");
        }
    }
    else if (mHasGoal)   // If there is a specified moving target, call PF-RRT* to find a solution.
    {
        findSolution();
        init_time_cost = 0.0;
    }
    else   // The expansion of tree will stop after the process of initialization takes more than 1s.
    {
        RCLCPP_WARN(get_logger(),
                    "The tree is large enough. Stop expansion! Current size: %d",
                    static_cast<int>(mPFRRTStar->tree().size()));
    }
}
