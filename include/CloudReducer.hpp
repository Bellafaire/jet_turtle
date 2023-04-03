// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/features/normal_3d.h>


// Namespace matches ROS package name
namespace jet_turtle
{

    class CloudReducer
    {
    public:
        CloudReducer(ros::NodeHandle &n, ros::NodeHandle &pn);

    private:
        void cloud_callback(const sensor_msgs::PointCloud2 &msg);

        ros::Subscriber cloud_sub;
        ros::Publisher cloud_pub;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        // KD search tree object for use by PCL functions
        pcl::search::Search<pcl::PointXYZRGB>::Ptr kd_tree_;
    };

}

