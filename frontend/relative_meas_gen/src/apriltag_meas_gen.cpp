#include <relative_meas_gen/apriltag_meas_gen.h>

void ApriltagMeasurer::imageCallback(/* Fill in type */) {

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
    robot_images = nh_.subscribe("/wilbur_left/", 1, &ApriltagMeasurer::imageCallback, this);

    relative_meas = nh.advertise<std_msgs::Int32.h>("/relative_meas/", 10);
    std::cout << "Hello World from Apriltag" << std::endl;
}

float /* Or RT matrix? */ ApriltagMeasurer::getRelativeMeasurement(/* Probably RT matrices */) {

}