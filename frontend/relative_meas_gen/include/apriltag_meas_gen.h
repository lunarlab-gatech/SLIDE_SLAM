#pragma once

#include <relative_meas_gen/slidetag.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

class ApriltagMeasurer {
    public:
        explicit ApriltagMeasurer(ros::NodeHandle nh);
    private:
        ros::Subscriber robot_images;
        ros::Publisher relative_meas;

        void imageCallback(/* Fill in message information */);
        float /* Or RT matrix? */ getRelativeMeasurement(/* Some parameters (RT matrices?) */);

        ros::NodeHandle nh_;
}