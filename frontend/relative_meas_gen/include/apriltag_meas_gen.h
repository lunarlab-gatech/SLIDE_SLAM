#pragma once

#include <apriltag_wrapper.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Pose.h>
#include <sloam_msgs/RelativeInterRobotMeasurement.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>


class ApriltagMeasurer {
    public:
        explicit ApriltagMeasurer(ros::NodeHandle nh);
    private:
        ros::Subscriber robot_images_sub;
        ros::Publisher relative_meas_pub;
        std::string robot_ID;
        std::string camera_ID;
        float intrinsics[4];
        float tagsize;
        YAML::Node config;

        void imageCallback(const sensor_msgs::CompressedImage msg);
        void PublishRelativeMeasurement(int8_t, Eigen::Matrix4f transformation);
        Eigen::Matrix4d CalculateRelativeTransformation(Eigen::Matrix4d H_hostBot_to_cam, 
                      Eigen::Matrix4d H_cam_to_tag, Eigen::Matrix4d H_observedBot_to_tag);
        std::tuple<int8_t, std::array<Eigen::Matrix4f, 2>> LoadTransformations(apriltag_wrapper tag);

        ros::NodeHandle nh_;
};