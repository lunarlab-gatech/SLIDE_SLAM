#include <relative_meas_gen/apriltag_meas_gen.h>

ApriltagMeasurer::ApriltagMeasurer(ros::NodeHandle nh): nh_(nh) {
    robot_images = nh_.subscribe(/* TOPIC NAME */, 1, &ApriltagMeasurer::imageCallback, this);

    relative_meas = nh_.advertise</* Fill in type */>(/* TOPIC NAME */, 1);
}

void ApriltagMeasurer::imageCallback(/* Fill in type */) {

    cv::Mat img = MatFromImage(/* Image Data */);
    map<int, slidetag> = ExtractAprilTags(cv::Mat img /* Fill in camera parameters */);

    /*
    TODO:
    - Get information about which robot is looking
    - Find which robot is being looked at
    - Call relative measurement function
    */

}

float /* Or RT matrix? */ ApriltagMeasurer::getRelativeMeasurement(/* Probably RT matrices */) {

}