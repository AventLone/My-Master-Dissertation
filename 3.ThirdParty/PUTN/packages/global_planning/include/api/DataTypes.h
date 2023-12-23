/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>


namespace putn
{
const float gINF = std::numeric_limits<float>::max();

struct Node;
struct Plane;
struct World;

/**
 * @brief Struct for setting parameters when fitting local plane
 */
struct FitPlaneArg
{
    double w_total;
    double w_flatness;
    double w_slope;
    double w_sparsity;
    double ratio_max;
    double ratio_min;
    double conv_thre;
};

/**
 * @brief Class for describing the local plane.The normal vector can be used to estimate the real
 * position of the robot when moving on this plane,and the traversability can evaluate whether the
 * the robot is safe when moving on this plane.
 */
struct Plane
{
    using Ptr = std::shared_ptr<Plane>;
    explicit Plane() = default;
    explicit Plane(const Eigen::Vector3d& p_surface, std::shared_ptr<World> world, double radius,
                   const FitPlaneArg& arg);
    ~Plane() = default;

    Eigen::Vector2d init_coord;
    std::vector<Eigen::Vector3d> plane_points;
    Eigen::Vector3d normal_vector;
    float traversability;
};

/**
 * @brief Class for storing the info of vertex.The recorded information of parent and child nodes will be used to bulid
 * the tree.The cost value represents the sum of the cost values required to arrive from the root node of the tree.
 */
struct Node
{
    using Ptr = std::shared_ptr<Node>;
    using ConstPtr = std::shared_ptr<const Node>;
    explicit Node() = default;
    Node(const Node& node);
    ~Node() = default;

    std::vector<Node::Ptr> children;

    Node::Ptr parent{nullptr};

    Eigen::Vector3d position;

    float cost{0.0f};

    Plane::Ptr plane{nullptr};
};

/**
 * @brief Class for storing information about a path from the start point to the end point.Besides
 * the set of nodes that describe the path,it also saves the Eucildean length and the cost value of
 * the path. The member variable `type` is used to indecate whether the end point of this path is a
 * real goal or a temporary sub goal
 */
struct Path
{
    using Ptr = std::shared_ptr<Path>;
    std::vector<Node::Ptr> nodes;
    float dis;
    float cost{gINF};
    enum Type
    {
        Global,
        Sub,
        Empty
    } type{Empty};
};


// namespace visualization
// {
// void visualizeWorld(World* world, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& world_vis_pub);
// }

/**
 * @brief Class for storing obstacles and world dimension.The information of obstacle is stored in a three-dimensional
 * bool array. Before using the PF-RRT* algorithm,a suitable grid map must be built
 */
struct World
{
    using Ptr = std::shared_ptr<World>;
    using ConstPtr = std::shared_ptr<const World>;

    World(float resolution = 0.1f) : mResolution(resolution)
    {
        mLowerBound = gINF * Eigen::Vector3d::Ones();
        mUpperBound = -gINF * Eigen::Vector3d::Ones();
        index_count = Eigen::Vector3i::Zero();
    }

    ~World()
    {
        clearMap();
    }

    // friend void visualization::visualizeWorld(World* world, ros::Publisher* world_vis_pub);

    Eigen::Vector3i index_count;

    bool exist_map{false};   // indicate whether the range of the grid map has been determined

    bool*** grid_map{nullptr};   // 3 dimentional occupancy map


    /**
     * @brief Automatically determine the upperbound and lowerbound of the grid map according to the
     *        information of the input point cloud.
     */
    void initGridMap(const pcl::PointCloud<pcl::PointXYZ>& cloud);

    /**
     * @brief Manually specify the upperbound and lowerbound.
     */
    void initGridMap(const Eigen::Vector3d& lowerbound, const Eigen::Vector3d& upperbound);
    void setObstacle(const Eigen::Vector3d& point)
    {
        Eigen::Vector3i idx = coord2index(point);
        grid_map[idx(0)][idx(1)][idx(2)] = false;
    }

    /**
     * @brief Find the grid closet to the point and return the coordinate of its center
     */
    Eigen::Vector3d coordRounding(const Eigen::Vector3d& coord)
    {
        return index2coord(coord2index(coord));
    }

    bool isFree(const Eigen::Vector3d& point)
    {
        Eigen::Vector3i idx = coord2index(point);
        bool is_free = isInsideBorder(idx) && grid_map[idx(0)][idx(1)][idx(2)];
        return is_free;
    }
    bool isFree(const float& coord_x, const float& coord_y, const float& coord_z)
    {
        return isFree(Eigen::Vector3d(coord_x, coord_y, coord_z));
    }

    /**
     * @brief Given a 2D coord, start from the lowerbound of the height of the grid map,search upward,
     *        and determine the boundary between the occupied area and the non occupied area as the
     *        surface point.
     * @param x the first dimension
     * @param y the second dimension
     * @param p_surface store the result of the projecting
     * @return bool true(no obstacle exists),false(exist obstacle)
     */
    bool project2surface(float x, float y, Eigen::Vector3d& p_surface);
    bool project2surface(const Eigen::Vector3d& p_original, Eigen::Vector3d& p_surface)
    {
        return project2surface(p_original(0), p_original(1), p_surface);
    }

    /**
     * @brief Check if there is any obstacle between 2 nodes.
     * @param node_start
     * @param node_end
     * @return bool true(no obstacle exists),false(exist obstacle)
     */
    bool collisionFree(const Node::ConstPtr& node_start, const Node::ConstPtr& node_end);

    /**
     * @brief Check whether the given point is within the range of the grid map
     * @param index the index value obtained after discretization of the given point
     * @return bool true(within range),false（out of range)
     */
    bool isInsideBorder(const Eigen::Vector3i& index)
    {
        return index(0) >= 0 && index(1) >= 0 && index(2) >= 0 && index(0) < index_count(0) &&
               index(1) < index_count(1) && index(2) < index_count(2);
    }
    bool isInsideBorder(const Eigen::Vector3d& point)
    {
        return isInsideBorder(coord2index(point));
    }

    /**
     * @brief get the low bound of the world
     * @return Vector3d
     */
    Eigen::Vector3d getLowerBound()
    {
        return mLowerBound;
    }

    /**
     * @brief get the up bound of the world
     * @return Vector3d
     */
    Eigen::Vector3d getUpperBound()
    {
        return mUpperBound;
    }

    /**
     * @brief get resolution of the world
     * @return float
     */
    float getResolution()
    {
        return mResolution;
    }

    Eigen::Vector3d index2coord(const Eigen::Vector3i& index)
    {
        Eigen::Vector3d coord =
            mResolution * index.cast<double>() + mLowerBound + 0.5 * mResolution * Eigen::Vector3d::Ones();
        return coord;
    }

    Eigen::Vector3i coord2index(const Eigen::Vector3d& coord)
    {
        Eigen::Vector3i index = ((coord - mLowerBound) / mResolution).cast<int>();
        return index;
    }

    void clearMap();

private:
    float mResolution;
    Eigen::Vector3d mLowerBound, mUpperBound;
};

/**
 * @brief Given a 3D point,extract its x and y coordinates and return a 2D point
 * @param p the 3D point
 * @return the 2D coordinate containing only x, y of p
 */
inline Eigen::Vector2d project2plane(const Eigen::Vector3d& p)
{
    return Eigen::Vector2d(p(0), p(1));
}
inline Eigen::Vector2d project2plane(float x, float y)
{
    return Eigen::Vector2d(x, y);
}

inline float EuclideanDistance(const Eigen::VectorXd& p, const Eigen::VectorXd& q)
{
    return (p - q).norm();
}
inline float EuclideanDistance(const Node::ConstPtr& p, const Node::ConstPtr& q)
{
    return EuclideanDistance(p->position, q->position);
}

inline float calCostBetweenTwoNode(const Node::ConstPtr& n1, const Node::ConstPtr& n2)
{
    float dis = EuclideanDistance(n1, n2);
    float cost =
        dis * (1.0f + 0.1f * (1 / (1.0001f - n1->plane->traversability) + 1 / (1.0001f - n2->plane->traversability)));
    return cost;
}
}   // namespace putn
