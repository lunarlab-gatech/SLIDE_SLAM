#pragma once

#include <slidetag.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>


class ApriltagMeasurer {
    public:
        explicit ApriltagMeasurer(ros::NodeHandle nh);
    private:
        ros::Subscriber robot_images;
        ros::Publisher relative_meas;

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        float /* Or RT matrix? */ getRelativeMeasurement(/* Some parameters (RT matrices?) */);

        ros::NodeHandle nh_;
};