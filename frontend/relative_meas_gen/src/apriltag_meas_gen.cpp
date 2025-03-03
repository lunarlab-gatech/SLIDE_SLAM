#include "apriltag_meas_gen.h"

void ApriltagMeasurer::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    //cv::Mat img = MatFromImage(/* Image Data */);
    //map<int, slidetag> = ExtractAprilTags(cv::Mat img /* Fill in camera parameters */);

    /*
    TODO:
    - Get information about which robot is looking
    - Find which robot is being looked at
    - Call relative measurement function
    */
   std::cout << "I recieved an image!" << std::endl;

}

ApriltagMeasurer::ApriltagMeasurer(ros::NodeHandle nh): nh_(nh) {
    robot_images = nh_.subscribe("/wilbur_left", 1, &ApriltagMeasurer::imageCallback, this);

    relative_meas = nh_.advertise<std_msgs::Int32>("/relative_meas/", 10);
    std::cout << "Hello World from Apriltag" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_node");
    ros::NodeHandle nh;
    ApriltagMeasurer measurer(nh);
    ros::spin();
    return 0;
}

float /* Or RT matrix? */ ApriltagMeasurer::getRelativeMeasurement(/* Probably RT matrices */) {

}