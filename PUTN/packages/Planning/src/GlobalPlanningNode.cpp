#include "GlobalPlanningNode.h"

GlobalPlanningNode::GlobalPlanningNode(const std::string& name) : rclcpp::Node(name)
{
    declare_parameter("Map.Resolution", 0.1);
    declare_parameter("Plannig.GoalThre", 1.0);
    declare_parameter("Plannig.StepSize", 0.2);
    declare_parameter("Plannig.CartHeight", 0.4);   // The height from surface to cart bottom
    declare_parameter("Plannig.NeighborRadius", 1.0);

    declare_parameter("Plannig.FitPlane", 0.4);
    declare_parameter("Plannig.Flatness", 4000.0);
    declare_parameter("Plannig.Slope", 0.4);
    declare_parameter("Plannig.Sparsity", 0.4);
    declare_parameter("Plannig.RatioMin", 0.25);
    declare_parameter("Plannig.RatioMax", 0.4);
    declare_parameter("Plannig.ConvThre", 0.1152);
    declare_parameter("Plannig.FitPlaneRadius", 1.0);
    declare_parameter("Plannig.MaxInitTime", 1000.0);


    mMapSub = create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_map",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&GlobalPlanningNode::cloudMapCallback, this, std::placeholders::_1));

    mWayPointSub = create_subscription<nav_msgs::msg::Path>(
        "waypoints",
        rclcpp::SensorDataQoS().reliable(),
        std::bind(&GlobalPlanningNode::waypointsCallback, this, std::placeholders::_1));

    mGridMapVisualizePub =
        create_publisher<sensor_msgs::msg::PointCloud2>("grid_map_vis", rclcpp::SensorDataQoS().reliable());
    mPathVisualizePub =
        create_publisher<visualization_msgs::msg::Marker>("path_vis", rclcpp::SensorDataQoS().reliable());
    mGoalVisualizePub =
        create_publisher<visualization_msgs::msg::Marker>("goal_vis", rclcpp::SensorDataQoS().reliable());
    mPlaneVisualizePub =
        create_publisher<sensor_msgs::msg::PointCloud2>("surf_vis", rclcpp::SensorDataQoS().reliable());
    mTreeVisualizePub =
        create_publisher<visualization_msgs::msg::Marker>("tree_vis", rclcpp::SensorDataQoS().reliable());
    mPathInterpolationVisualizePub =
        create_publisher<std_msgs::msg::Float32MultiArray>("tree_tra", rclcpp::SensorDataQoS().reliable());
    mTreeTraVisualizePub =
        create_publisher<std_msgs::msg::Float32MultiArray>("global_path", rclcpp::SensorDataQoS().reliable());

    // nh.param("map/resolution", resolution, 0.1);

    // nh.param("planning/goal_thre", goal_thre, 1.0);
    // nh.param("planning/step_size", step_size, 0.2);
    // nh.param("planning/h_surf_car", h_surf_car, 0.4);
    // nh.param("planning/neighbor_radius", neighbor_radius, 1.0);

    // nh.param("planning/w_fit_plane", fit_plane_arg.w_total, 0.4);
    // nh.param("planning/w_flatness", fit_plane_arg.w_flatness, 4000.0);
    // nh.param("planning/w_slope", fit_plane_arg.w_slope, 0.4);
    // nh.param("planning/w_sparsity", fit_plane_arg.w_sparsity, 0.4);
    // nh.param("planning/ratio_min", fit_plane_arg.ratio_min, 0.25);
    // nh.param("planning/ratio_max", fit_plane_arg.ratio_max, 0.4);
    // nh.param("planning/conv_thre", fit_plane_arg.conv_thre, 0.1152);

    // nh.param("planning/radius_fit_plane", radius_fit_plane, 1.0);

    // nh.param("planning/max_initial_time", max_initial_time, 1000.0);

    // // Initialization
    // world = new World(resolution);
    // pf_rrt_star = new PFRRTStar(h_surf_car, world);

    // // Set argument of PF-RRT*
    // pf_rrt_star->setGoalThre(goal_thre);
    // pf_rrt_star->setStepSize(step_size);
    // pf_rrt_star->setFitPlaneArg(fit_plane_arg);
    // pf_rrt_star->setFitPlaneRadius(radius_fit_plane);
    // pf_rrt_star->setNeighborRadius(neighbor_radius);

    // tf2::TransformListener listener;
}

void GlobalPlanningNode::waypointsCallback(const nav_msgs::msg::Path::ConstSharedPtr& wp)
{
    if (!mWorld->mExistMap) return;
    mHasGoal = true;
    target_pt =
        Eigen::Vector3d(wp->poses[0].pose.position.x, wp->poses[0].pose.position.y, wp->poses[0].pose.position.z);
    RCLCPP_INFO(get_logger(), "Receive the planning target");
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
    // visualizeWorld(world, &grid_map_vis_pub);
}

void GlobalPlanningNode::pubInterpolatedPath(const std::vector<putn::Node::Ptr>& solution)
{
    // if (path_interpolation_pub == NULL) return;
    std_msgs::msg::Float32MultiArray msg;
    for (size_t i = 0; i < solution.size(); i++)
    {
        if (i == solution.size() - 1)
        {
            msg.data.push_back(solution[i]->mPosition(0));
            msg.data.push_back(solution[i]->mPosition(1));
            msg.data.push_back(solution[i]->mPosition(2));
        }
        else
        {
            size_t interpolation_num = static_cast<size_t>(putn::EuclideanDistance(solution[i + 1], solution[i]) / 0.1);
            Eigen::Vector3d diff_pt = solution[i + 1]->mPosition - solution[i]->mPosition;
            for (size_t j = 0; j < interpolation_num; j++)
            {
                Eigen::Vector3d interpt = solution[i]->mPosition + diff_pt * (float)j / interpolation_num;
                msg.data.push_back(interpt(0));
                msg.data.push_back(interpt(1));
                msg.data.push_back(interpt(2));
            }
        }
    }
    mPathInterpolationVisualizePub->publish(msg);
}

void GlobalPlanningNode::findSolution()
{
    std::printf("=========================================================================\n");
    RCLCPP_INFO(get_logger(), "Start calling PF-RRT*");
    putn::Path solution = putn::Path();

    mPFRRTStar->initWithGoal(start_pt, target_pt);

    /* Case 1: The PF-RRT* can't work at when the origin can't be projected to surface. */
    if (mPFRRTStar->state() == putn::planner::Invalid)
    {
        RCLCPP_WARN(get_logger(), "The start point can't be projected. Unable to start PF-RRT* algorithm!!!");
    }

    /* Case 2: If both the origin and the target can be projected,the PF-RRT* will
       execute global planning and try to generate a path. */
    else if (mPFRRTStar->state() == putn::Path::Global)
    {
        RCLCPP_INFO(get_logger(), "Starting PF-RRT* algorithm at the state of global planning.");
        int max_iter = 5000;
        double max_time = 100.0;

        while (solution.mType == putn::Path::Empty && max_time < mMaxInitialTime)
        {
            solution = mPFRRTStar->planner(max_iter, max_time);
            max_time += 100.0;
        }

        if (!solution.mNodes.empty())
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
        RCLCPP_INFO(get_logger(), "Starting PF-RRT* algorithm at the state of rolling planning");
        int max_iter = 1500;
        double max_time = 100.0;

        solution = mPFRRTStar->planner(max_iter, max_time);

        if (!solution.mNodes.empty())
        {
            RCLCPP_INFO(get_logger(), "Get a sub path!");
        }
        else
        {
            RCLCPP_WARN(get_logger(), "No solution found!");
        }
    }
    RCLCPP_INFO(get_logger(), "End calling PF-RRT*");
    std::printf("=========================================================================\n");

    // pubInterpolatedPath(solution.mNodes, &path_interpolation_pub);
    // visualizePath(solution.mNodes, &path_vis_pub);
    // visualizePlane(solution.mNodes, &surf_vis_pub);

    /* When the PF-RRT* generates a short enough global path,
       it's considered that the robot has reached the goal region. */
    if (solution.mType == putn::Path::Global &&
        putn::EuclideanDistance(mPFRRTStar->origin(), mPFRRTStar->target()) < goal_thre)
    {
        mHasGoal = false;
        // visualizeOriginAndGoal({}, &goal_vis_pub);   // Passing an empty set to delete the previous display
        // visualizePath({}, &path_vis_pub);
        // ROS_INFO("The Robot has achieved the goal!!!");
    }

    if (solution.mType == putn::Path::Empty)
    {
        // visualizePath({}, &path_vis_pub);
    }
}

void GlobalPlanningNode::callPlanner()
{
    static double init_time_cost = 0.0;
    if (!mWorld->mExistMap) return;

    /* The tree will expand at a certain frequency to explore the space more fully. */
    if (!mHasGoal && init_time_cost < 1000.0)
    {
        mPFRRTStar->initWithoutGoal(start_pt);

        if (mPFRRTStar->state() == putn::planner::WithoutGoal)
        {
            int max_iter = 550;
            double max_time = 100.0;
            mPFRRTStar->planner(max_iter, max_time);
            RCLCPP_INFO(get_logger(), "Current size of tree: %d", static_cast<int>(mPFRRTStar->tree().size()));
        }
        else
        {
            RCLCPP_WARN(get_logger(), "The start point can't be projected, unable to execute PF-RRT* algorithm.");
        }
    }
    else if (mHasGoal)   // If there is a specified moving target, call PF-RRT* to find a solution.
    {
        findSolution();
        init_time_cost = 0.0;
    }
    else   // The expansion of tree will stop after the process of initialization takes more than 1s.
    {
        RCLCPP_INFO(get_logger(),
                    "The tree is large enough. Stop expansion! Current size: %d",
                    static_cast<int>(mPFRRTStar->tree().size()));
    }
}
