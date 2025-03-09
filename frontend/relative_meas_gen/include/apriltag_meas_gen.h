#pragma once

#include <slidetag.h>
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
        ros::Subscriber robot_images;
        ros::Publisher relative_meas;
        std::string robot_ID;
        std::string camera_ID;
        float intrinsics[4];
        float tagsize;
        YAML::Node config;

        void imageCallback(const sensor_msgs::CompressedImage msg);
        void PublishRelativeMeasurement(int8_t, Eigen::Matrix4f transformation);
        Eigen::Matrix4f CalculateRelativeTransformation(Eigen::Matrix4f bot_to_cam_RT, Eigen::Matrix4f cam_to_tag_RT, Eigen::Matrix4f bot_to_tag);
        Eigen::Matrix4f RollPitchYaw_to_RT(float x, float y, float z, float roll, float pitch, float yaw);
        std::tuple<int8_t, std::array<Eigen::Matrix4f, 2>> LoadTransformations(slidetag tag);

        ros::NodeHandle nh_;
};