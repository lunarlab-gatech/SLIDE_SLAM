#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <apriltag_wrapper.h>

TEST(apriltag_test, Detect) {
    
    std::string path = "/home/cam/Documents/slideslam_docker_ws/src/SLIDE_SLAM/frontend/relative_meas_gen/test/apriltag_test_pic.jpg";
    
    //std::string package_path = ros::package::getPath("relative_meas_gen");
    //std::string path = package_path + "/test/apriltag_test_pic.jpg";
    

    std::cout << path << std::endl;
    cv::Mat matrix = cv::imread(path, cv::IMREAD_COLOR).clone();
    std::cout << matrix.cols << std::endl;
    cv::Mat image;
    cv::cvtColor(matrix, image, cv::COLOR_BGR2GRAY);
    
    double intrinsics[4];
    intrinsics[0] = 1903.520006386324;
    intrinsics[1] = 681.5060629940748;
    intrinsics[2] = 1895.619818759923;
    intrinsics[3] = 517.1128313302568;

    double tagsize = 0.17;

    std::vector<apriltag_wrapper> tags = ExtractAprilTags(image, intrinsics, tagsize);
    apriltag_wrapper test_tag = tags[0];
    std::cout << "Detected tag: " << test_tag.id << std::endl;
    std::cout << "Expected tag: 6" << std::endl;

    EXPECT_EQ(6, test_tag.id);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "apriltag_test");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
