#include "CloudReducer.hpp"
// ROS and node class header file
#include <ros/ros.h>

namespace jet_turtle
{

    CloudReducer::CloudReducer(ros::NodeHandle &n, ros::NodeHandle &pn) : tf_listener(tf_buffer), kd_tree_(new pcl::search::KdTree<pcl::PointXYZRGB>)
    {
        cloud_sub = n.subscribe("/realsense_camera/depth/color/points", 1, &CloudReducer::cloud_callback, this);
        cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/reduced_realsense", 1);
    }

    void CloudReducer::cloud_callback(const sensor_msgs::PointCloud2 &msg)
    {
        // ros msg -> PCL cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(msg, *cloud);

        // filter out any points beyond 3 meters
        pcl::IndicesPtr indices(new std::vector<int>);
        pcl::PassThrough<pcl::PointXYZRGB> filter;

        filter.setInputCloud(cloud);
        // filter.setIndices(indices);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(0, 3.0);
        // filter.filter(*indices);
        filter.filter(*cloud);

        // publish new cloud
        sensor_msgs::PointCloud2 new_cloud_msg;
        pcl::toROSMsg(*cloud, new_cloud_msg);
        new_cloud_msg.header = pcl_conversions::fromPCL(cloud->header);

        cloud_pub.publish(new_cloud_msg);
    }

}

// start the node
int main(int argc, char **argv)
{
    // Initialize ROS and declare node handles
    ros::init(argc, argv, "realsense_cloud_filter");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Instantiate node class
    jet_turtle::CloudReducer node(n, pn);

    // Spin and process callbacks
    ros::spin();
}
