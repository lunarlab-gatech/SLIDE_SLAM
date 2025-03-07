#pragma once

#include <slidetag.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>


class ApriltagMeasurer {
    public:
        explicit ApriltagMeasurer(ros::NodeHandle nh);
    private:
        ros::Subscriber robot_images;
        ros::Publisher relative_meas;
        std::string robot_ID;
        std::string camera_ID;
        float intrinsics[4];
        float tagsize;
        YAML::Node config;

        void imageCallback(const sensor_msgs::CompressedImage msg);
        void publishRelativeMeasurement(std::string, Eigen::Matrix4f bot_to_cam_RT, Eigen::Matrix4f cam_to_tag_RT, Eigen::Vector3f bot_to_tag_T, Eigen::Quaternionf bot_to_tag_Q);

        ros::NodeHandle nh_;
};