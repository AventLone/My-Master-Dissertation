#include "GlobalPlanningNode.h"
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/**
 * @brief Make frame to show pose and shape
 */
std::vector<Eigen::Vector4d> generateFrame(const std::vector<Eigen::Vector3d>& pts, const std::vector<float>& color_pts,
                                           const std::vector<Eigen::Quaterniond>& orientations);

void GlobalPlanningNode::visualizeWorld()
{
    if (!mWorld->exist_map) return;

    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    for (int i = 0; i < mWorld->index_count(0); i++)
    {
        for (int j = 0; j < mWorld->index_count(1); j++)
        {
            for (int k = 0; k < mWorld->index_count(2); k++)
            {
                Eigen::Vector3i index(i, j, k);
                if (!mWorld->grid_map[index(0)][index(1)][index(2)])
                {
                    Eigen::Vector3d coor_round = mWorld->index2coord(index);
                    pcl::PointXYZ pt_add;
                    pt_add.x = coor_round(0);
                    pt_add.y = coor_round(1);
                    pt_add.z = coor_round(2);
                    cloud_vis.points.push_back(pt_add);
                }
            }
        }
    }

    cloud_vis.width = cloud_vis.points.size();
    cloud_vis.height = 1;
    cloud_vis.is_dense = true;

    sensor_msgs::msg::PointCloud2 map_vis;
    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "world";
    mGridMapVisualizePub->publish(map_vis);
}

void GlobalPlanningNode::visualizePlane(const std::vector<putn::Node::Ptr>& solution)
{
    pcl::PointCloud<pcl::PointXYZRGB> surf_points;
    pcl::PointXYZRGB pt;
    for (const auto& node : solution)
    {
        for (const auto& point : node->plane->plane_points)
        {
            pt.x = point(0);
            pt.y = point(1);
            pt.z = point(2);
            pt.r = pt.g = pt.b = 0;
            pt.a = 1.0f;

            surf_points.push_back(pt);
        }
    }

    surf_points.width = surf_points.points.size();
    surf_points.height = 1;
    surf_points.is_dense = true;

    sensor_msgs::msg::PointCloud2 map_vis;
    pcl::toROSMsg(surf_points, map_vis);

    map_vis.header.frame_id = "world";
    mPlaneVisualizePub->publish(map_vis);
}

void GlobalPlanningNode::visualizeOriginAndGoal(const std::vector<putn::Node::Ptr>& nodes)
{
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = this->now();
    sphere.ns = "Goal";
    if (nodes.empty())
    {
        sphere.action = visualization_msgs::msg::Marker::DELETE;
        mGoalVisualizePub->publish(sphere);
        return;
    }
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.orientation.w = 1.0;
    sphere.id = 0;
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;

    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2f;

    sphere.color.g = sphere.color.b = sphere.color.r = sphere.color.a = 1.0f;

    geometry_msgs::msg::Point pt;
    for (const auto& node : nodes)
    {
        Eigen::Vector3d coord = node->position;
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        sphere.points.push_back(pt);
    }
    mGoalVisualizePub->publish(sphere);
}

void GlobalPlanningNode::visualizePath(const std::vector<putn::Node::Ptr>& solution)
{
    visualization_msgs::msg::Marker Points, Line, Frame;
    Frame.header.frame_id = Points.header.frame_id = Line.header.frame_id = "world";
    Frame.header.stamp = Points.header.stamp = Line.header.stamp = this->now();
    Line.pose.orientation.w = Frame.pose.orientation.w = 1.0f;
    Frame.ns = Points.ns = Line.ns = "Path";
    Points.id = 0;
    Line.id = 1;
    Frame.id = 2;

    Points.type = visualization_msgs::msg::Marker::POINTS;
    Line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    Frame.type = visualization_msgs::msg::Marker::LINE_LIST;

    Frame.scale.x = 0.05;
    Points.scale.x = 0.1;
    Points.scale.y = 0.1;
    Line.scale.x = 0.1;

    Points.color.g = Points.color.a = 1.0f;
    Line.color.b = Line.color.a = 1.0f;
    Frame.color.g = Frame.color.b = Frame.color.a = 1.0f;

    if (solution.size() > 1)
    {
        std::vector<Eigen::Vector3d> pts;
        std::vector<float> pts_tra;
        std::vector<Eigen::Quaterniond> orientations;
        for (const auto& node : solution)
        {
            pts.push_back(node->position);
            pts_tra.push_back(node->plane->traversability);
        }

        geometry_msgs::msg::Point pt;
        for (const auto& coord : pts)
        {
            pt.x = coord(0);
            pt.y = coord(1);
            pt.z = coord(2);

            Points.points.push_back(pt);
            Line.points.push_back(pt);
        }

        for (size_t i = 0; i < solution.size() - 1; i++)
        {
            Eigen::Vector3d q = solution[i + 1]->position - solution[i]->position;
            Eigen::Vector3d e_z = solution[i]->plane->normal_vector;

            Eigen::Vector3d e_x = q - (q.dot(e_z)) * q;
            e_x.normalize();

            Eigen::Vector3d e_y = e_z.cross(e_x);

            Eigen::Matrix3d R;
            R << e_x(0), e_y(0), e_z(0), e_x(1), e_y(1), e_z(1), e_x(2), e_y(2), e_z(2);
            Eigen::Quaterniond quaternion(R);
            orientations.push_back(quaternion);
        }
        // make frame
        std::vector<Eigen::Vector4d> frame_list = generateFrame(pts, pts_tra, orientations);
        for (const auto& coord : frame_list)
        {
            pt.x = coord(0);
            pt.y = coord(1);
            pt.z = coord(2);
            std_msgs::msg::ColorRGBA color;
            color.r = 1;
            color.g = coord(3);
            color.b = 0.05;
            color.a = 0.8;

            Frame.points.push_back(pt);
            Frame.colors.push_back(color);
        }
    }
    else
    {
        Frame.action = Points.action = Line.action = visualization_msgs::msg::Marker::DELETE;
    }

    mPathVisualizePub->publish(Points);
    mPathVisualizePub->publish(Line);
    mPathVisualizePub->publish(Frame);
}

void GlobalPlanningNode::visualizeTree(const std::vector<putn::Node::Ptr>& tree)
{
    visualization_msgs::msg::Marker Points, Line;
    Points.header.frame_id = Line.header.frame_id = "world";
    Points.header.stamp = Line.header.stamp = this->now();
    Points.ns = Line.ns = "Tree";
    Points.action = Line.action = visualization_msgs::msg::Marker::ADD;
    Points.pose.orientation.w = Line.pose.orientation.w = 1.0f;
    Points.id = 0;
    Line.id = 1;

    Points.type = visualization_msgs::msg::Marker::POINTS;
    Line.type = visualization_msgs::msg::Marker::LINE_LIST;

    Points.scale.x = Points.scale.y = 0.05;
    Line.scale.x = 0.01;

    Points.color.g = Points.color.a = 0.5f;   // Points are green and Line Strip is blue
    // Points.color.g = Points.color.r = 255*traversability;
    Line.color.b = Line.color.a = 0.75f;

    geometry_msgs::msg::Point pt;
    geometry_msgs::msg::Point parent_pt;
    for (const auto& node : tree)
    {
        pt.x = node->position(0);
        pt.y = node->position(1);
        pt.z = node->position(2);
        // std_msgs::msg::ColorRGBA color;
        // color.r=color.g=node->plane->traversability;
        // color.b=0;
        // color.a=1;

        // Points.colors.push_back(color);
        Points.points.push_back(pt);

        if (node->parent != NULL)   // skip the root node
        {
            Line.points.push_back(pt);
            parent_pt.x = node->parent->position(0);
            parent_pt.y = node->parent->position(1);
            parent_pt.z = node->parent->position(2);
            Line.points.push_back(parent_pt);
        }
    }
    mTreeVisualizePub->publish(Points);
    mTreeVisualizePub->publish(Line);
}

std::vector<Eigen::Vector4d> generateFrame(const std::vector<Eigen::Vector3d>& pts, const std::vector<float>& color_pts,
                                           const std::vector<Eigen::Quaterniond>& orientations)
{
    float frame_w = 0.75;
    float frame_h = 0.36;

    std::vector<Eigen::Vector4d> Frame_list;
    geometry_msgs::msg::Point p1;
    geometry_msgs::msg::Point p2;
    geometry_msgs::msg::Point p3;
    geometry_msgs::msg::Point p4;

    for (size_t i = 1; i < pts.size() - 1; i++)
    {
        Eigen::Vector3d p1_tmp, p2_tmp, p3_tmp, p4_tmp;
        Eigen::Vector3d p1_, p2_, p3_, p4_;
        Eigen::Vector4d _p1, _p2, _p3, _p4;
        p1_tmp(0) = 0;
        p1_tmp(1) = +frame_w * 0.5;
        p1_tmp(2) = +frame_h * 0.5;
        p2_tmp(0) = 0;
        p2_tmp(1) = -frame_w * 0.5;
        p2_tmp(2) = +frame_h * 0.5;
        p3_tmp(0) = 0;
        p3_tmp(1) = +frame_w * 0.5;
        p3_tmp(2) = -frame_h * 0.5;
        p4_tmp(0) = 0;
        p4_tmp(1) = -frame_w * 0.5;
        p4_tmp(2) = -frame_h * 0.5;

        Eigen::Matrix3d ratation_matrix = orientations[i].matrix();

        p1_ = ratation_matrix * p1_tmp + pts[i];
        p2_ = ratation_matrix * p2_tmp + pts[i];
        p3_ = ratation_matrix * p3_tmp + pts[i];
        p4_ = ratation_matrix * p4_tmp + pts[i];

        _p1 << p1_(0), p1_(1), p1_(2), color_pts[i];
        _p2 << p2_(0), p2_(1), p2_(2), color_pts[i];
        _p3 << p3_(0), p3_(1), p3_(2), color_pts[i];
        _p4 << p4_(0), p4_(1), p4_(2), color_pts[i];

        Frame_list.push_back(_p1);
        Frame_list.push_back(_p2);
        Frame_list.push_back(_p2);
        Frame_list.push_back(_p4);
        Frame_list.push_back(_p4);
        Frame_list.push_back(_p3);
        Frame_list.push_back(_p3);
        Frame_list.push_back(_p1);
    }
    return Frame_list;
}
