#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <apriltag_wrapper.h>

TEST(apriltag_test, Detect) {
    
    std::string package_path = ros::package::getPath("relative_meas_gen");
    std::string image_path = package_path + "/tests/apriltag_test_pic.jpg";
    
    cv::Mat matrix = cv::imread(image_path, cv::IMREAD_COLOR).clone();
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
    EXPECT_EQ(6, test_tag.id);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "apriltag_test");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
