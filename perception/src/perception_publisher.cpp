#include "ros/ros.h"
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc,char** argv){
    ros::init(argc,argv,"perception_publisher");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Publisher point_cloud_publisher=nh.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/image_raw",2,true);

    rosbag::Bag bagfile;
    std::string path = ros::package::getPath("perception");
    path += "/doc/perception_pipeline/bags/perception_tutorial.bag";
    bagfile.open(path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back("/camera/depth_registered/image_raw");

    // Iterator for topics in bag.
    rosbag::View bag(bagfile, rosbag::TopicQuery(topics));

    sensor_msgs::PointCloud2::Ptr point_cloud_ptr = bag.begin()->instantiate<sensor_msgs::PointCloud2>();

    if(!point_cloud_ptr){
        ROS_FATAL("invalid message in rosbag");
        return 1;
    }

    ros::Duration(1.0).sleep();

    ros::Rate loop_rate(0.2);
    while(ros::ok()){
        point_cloud_ptr->header.stamp=ros::time::now();
        point_cloud_publisher.publish(*point_cloud_ptr);
        loop_rate.sleep();
    }
    bagfile.close();
    return 0;
}