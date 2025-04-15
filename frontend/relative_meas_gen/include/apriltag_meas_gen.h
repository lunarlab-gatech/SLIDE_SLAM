#pragma once

#include <apriltag_wrapper.h>
#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Pose.h>
#include <sloam_msgs/RelativeInterRobotMeasurement.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Transform.h>


class ApriltagMeasurer {
    public:
        explicit ApriltagMeasurer(ros::NodeHandle nh);
    private:
        ros::Subscriber robot_images_sub;
        ros::Publisher relative_meas_pub;
        int robot_ID;
        std::string camera_ID;
        double intrinsics[4];
        double dist_coefficients[4];
        double tagsize;
        YAML::Node config;
        Eigen::Matrix4d bot_to_cam;

        void imageCallback(const sensor_msgs::CompressedImage msg);
        void PublishRelativeMeasurement(int8_t, Eigen::Matrix4d transformation);
        Eigen::Matrix4d CalculateRelativeTransformation(Eigen::Matrix4d H_hostBot_to_cam, 
                      Eigen::Matrix4d H_cam_to_tag, Eigen::Matrix4d H_observedBot_to_tag);
        std::tuple<int8_t, Eigen::Matrix4d> LoadTransformations(apriltag_wrapper tag);

        ros::NodeHandle nh_;
};