#include <gtest/gtest.h>
#include <utils.h>

TEST(UtilsTest, SE3ToGTSAMPose3) {
    // Define a sample SE3 transformation
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d t(1.0, 2.0, 3.0);
    Sophus::SE3d se3_pose(R, t);
    
    // Convert using the function
    gtsam::Pose3 gtsam_pose = SE3ToGTSAMPose3(se3_pose);
    
    // Extract values from the output gtsam::Pose3
    gtsam::Rot3 gtsam_rot = gtsam_pose.rotation();
    gtsam::Point3 gtsam_trans = gtsam_pose.translation();
    
    // Check rotation matrix equality
    Eigen::Matrix3d expected_R = gtsam_rot.matrix();
    EXPECT_TRUE(expected_R.isApprox(R, 1e-15)) << "Rotation matrices do not match!";
    
    // Check translation vector equality
    EXPECT_NEAR(gtsam_trans.x(), t[0], 1e-15);
    EXPECT_NEAR(gtsam_trans.y(), t[1], 1e-15);
    EXPECT_NEAR(gtsam_trans.z(), t[2], 1e-15);
}

int main(int argc, char **argv) {
    // Initialize GoogleTest and run all tests
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}