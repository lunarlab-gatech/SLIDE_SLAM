#include <deque>
#include <gtest/gtest.h>
#include <limits>
#include "ros/time.h"
#include "sloam.h" 

class SloamTest : public ::testing::Test {
protected:
    // GetIndexClosestPoseMstPair
    int indexClosest;
    double timeDiffClosest;
    sloam::sloam sloamInstance;
    std::deque<PoseMstPair> poseMstPacket;

    // FindRelativeMeasurementMatch
    std::unordered_map<size_t, robotData> robotDataDict_;
    std::vector<RelativeMeasMatch> matches;
};

TEST_F(SloamTest, GetIndexClosestPoseMstPair) {
  // Test with an empty deque
  ros::Time stamp(10, 0);
  sloamInstance.GetIndexClosestPoseMstPair(poseMstPacket, stamp, indexClosest, timeDiffClosest);
  EXPECT_EQ(indexClosest, -1);
  EXPECT_DOUBLE_EQ(timeDiffClosest, std::numeric_limits<double>::max());

  // Test with a single element
  PoseMstPair p1; 
  p1.stamp = ros::Time(5, 0);
  poseMstPacket.push_back(p1);
  
  sloamInstance.GetIndexClosestPoseMstPair(poseMstPacket, stamp, indexClosest, timeDiffClosest);
  EXPECT_EQ(indexClosest, 0);
  EXPECT_DOUBLE_EQ(timeDiffClosest, 5.0);

  // Test with multiple elements
  p1.stamp = ros::Time(5, 0);
  PoseMstPair p2; p2.stamp = ros::Time(15, 0);
  PoseMstPair p3; p3.stamp = ros::Time(12, 0);
  poseMstPacket = {p1, p2, p3};
  
  stamp = ros::Time(11, 0);
  sloamInstance.GetIndexClosestPoseMstPair(poseMstPacket, stamp, indexClosest, timeDiffClosest);
  EXPECT_EQ(indexClosest, 2);
  EXPECT_DOUBLE_EQ(timeDiffClosest, 1.0);

  // Test tie-breaking (choosing the first occurrence)
  stamp = ros::Time(13, 500000000);
  sloamInstance.GetIndexClosestPoseMstPair(poseMstPacket, stamp, indexClosest, timeDiffClosest);
  EXPECT_EQ(indexClosest, 1);
  EXPECT_DOUBLE_EQ(timeDiffClosest, 1.5);

  stamp = ros::Time(13, 400000000);
  sloamInstance.GetIndexClosestPoseMstPair(poseMstPacket, stamp, indexClosest, timeDiffClosest);
  EXPECT_EQ(indexClosest, 2);
  EXPECT_DOUBLE_EQ(timeDiffClosest, 1.4);
};

TEST_F(SloamTest, FindRelativeMeasurementMatch) {
    // Define host robot
    int hostRobotID = 0;

    // Initialize all values to empty
    std::vector<size_t> pose_counter_robot_(2, 0);
    robotData hostRobotData;
    robotData otherRobotData;
    robotDataDict_[0] = hostRobotData;
    robotDataDict_[1] = otherRobotData;
    sloamInstance.feasible_relative_meas_for_factors.clear();

    // Test with empty input
    sloamInstance.FindRelativeMeasurementMatch(pose_counter_robot_, robotDataDict_, hostRobotID, matches);
    EXPECT_TRUE(matches.empty());

    // Test to make sure appropriate errors are raised for when robotIndex == hostRobotID
    RelativeMeas meas;
    meas.robotIndex = 0;
    sloamInstance.feasible_relative_meas_for_factors.push_back(meas);
    EXPECT_THROW(sloamInstance.FindRelativeMeasurementMatch(pose_counter_robot_, robotDataDict_, 
                                                      hostRobotID, matches), std::runtime_error);
    sloamInstance.feasible_relative_meas_for_factors.clear();

    // Test to make sure error is raised when useOnlyOdom == true
    meas = RelativeMeas();
    meas.robotIndex = 1;
    meas.onlyUseOdom = true;
    sloamInstance.feasible_relative_meas_for_factors.push_back(meas);
    EXPECT_THROW(sloamInstance.FindRelativeMeasurementMatch(pose_counter_robot_, robotDataDict_, 
                                                       hostRobotID, matches), std::runtime_error);
    sloamInstance.feasible_relative_meas_for_factors.clear();

    // Test with single relative measurement and no matching poses
    meas.stamp = ros::Time(5, 0);
    meas.onlyUseOdom = false;
    sloamInstance.feasible_relative_meas_for_factors.push_back(meas);

    sloamInstance.FindRelativeMeasurementMatch(pose_counter_robot_, robotDataDict_, hostRobotID, matches);
    EXPECT_TRUE(matches.empty());

    // Test with single relative measurement with a match
    PoseMstPair hostPose;
    hostPose.stamp = ros::Time(5, 0);
    hostRobotData.poseMstPacket.push_back(hostPose);
    pose_counter_robot_[0] = 1;

    PoseMstPair otherPose;
    otherPose.stamp = ros::Time(5, 0);
    otherRobotData.poseMstPacket.push_back(otherPose);
    pose_counter_robot_[1] = 1;

    robotDataDict_[0] = hostRobotData;
    robotDataDict_[1] = otherRobotData;

    sloamInstance.FindRelativeMeasurementMatch(pose_counter_robot_, robotDataDict_, hostRobotID, matches);
    EXPECT_EQ(matches.size(), 1);
    EXPECT_EQ(matches[0].indexClosestHostRobot, 0);
    EXPECT_EQ(matches[0].indexClosestOtherRobot, 0);
    EXPECT_EQ(matches[0].meas.stamp.toSec(), meas.stamp.toSec());
    EXPECT_EQ(sloamInstance.feasible_relative_meas_for_factors.size(), 0);

    // Test with multiple measurements
    matches.clear();
    sloamInstance.feasible_relative_meas_for_factors.clear();

    RelativeMeas meas2;
    meas2.stamp = ros::Time(7, 1000);
    meas2.robotIndex = 1;
    meas2.onlyUseOdom = false;
    sloamInstance.feasible_relative_meas_for_factors.push_back(meas);
    sloamInstance.feasible_relative_meas_for_factors.push_back(meas2);

    PoseMstPair hostPose2;
    hostPose2.stamp = ros::Time(7, 0);
    hostRobotData.poseMstPacket.push_back(hostPose2);
    pose_counter_robot_[0]++;

    PoseMstPair otherPose2;
    otherPose2.stamp = ros::Time(7, 0);
    otherRobotData.poseMstPacket.push_back(otherPose2);
    pose_counter_robot_[1]++;

    robotDataDict_[0] = hostRobotData;
    robotDataDict_[1] = otherRobotData;

    sloamInstance.FindRelativeMeasurementMatch(pose_counter_robot_, robotDataDict_, hostRobotID, matches);
    EXPECT_EQ(matches.size(), 2);
    EXPECT_EQ(matches[0].indexClosestHostRobot, 0);
    EXPECT_EQ(matches[0].indexClosestOtherRobot, 0);
    EXPECT_EQ(matches[0].meas.stamp.toSec(), meas.stamp.toSec());
    EXPECT_EQ(matches[1].indexClosestHostRobot, 1);
    EXPECT_EQ(matches[1].indexClosestOtherRobot, 1);
    EXPECT_EQ(matches[1].meas.stamp.toSec(), meas2.stamp.toSec());
    EXPECT_EQ(sloamInstance.feasible_relative_meas_for_factors.size(), 0);

    // Test not adding measurement exceeding time threshold (1ms)
    matches.clear();
    sloamInstance.feasible_relative_meas_for_factors.clear();

    RelativeMeas meas3;
    meas3.stamp = ros::Time(10, 0);
    meas3.robotIndex = 1;
    meas3.onlyUseOdom = false;
    sloamInstance.feasible_relative_meas_for_factors.push_back(meas3);

    PoseMstPair latehostPose;
    latehostPose.stamp = ros::Time(9, 8000000);
    hostRobotData.poseMstPacket.push_back(latehostPose);
    pose_counter_robot_[0]++;

    PoseMstPair lateOtherPose;
    lateOtherPose.stamp = ros::Time(10, 2000000);
    otherRobotData.poseMstPacket.push_back(lateOtherPose);
    pose_counter_robot_[1]++;
    
    robotDataDict_[0] = hostRobotData;
    robotDataDict_[1] = otherRobotData;

    sloamInstance.FindRelativeMeasurementMatch(pose_counter_robot_, robotDataDict_, hostRobotID, matches);
    EXPECT_TRUE(matches.empty());
    EXPECT_EQ(sloamInstance.feasible_relative_meas_for_factors.size(), 1);

    // Test removing outdated measurements
    matches.clear();
    sloamInstance.feasible_relative_meas_for_factors.clear();

    RelativeMeas staleMeas;
    staleMeas.stamp = ros::Time(2, 0);
    staleMeas.robotIndex = 1;
    staleMeas.onlyUseOdom = false;
    sloamInstance.feasible_relative_meas_for_factors.push_back(staleMeas);

    hostPose.stamp = ros::Time(4, 0);
    otherPose.stamp = ros::Time(4, 0);
    hostRobotData.poseMstPacket = {hostPose};
    otherRobotData.poseMstPacket = {otherPose};

    pose_counter_robot_[0] = 1;
    pose_counter_robot_[1] = 1;
    robotDataDict_[0] = hostRobotData;
    robotDataDict_[1] = otherRobotData;

    sloamInstance.FindRelativeMeasurementMatch(pose_counter_robot_, robotDataDict_, hostRobotID, matches);
    EXPECT_TRUE(matches.empty());
    EXPECT_TRUE(sloamInstance.feasible_relative_meas_for_factors.empty());
};


int main(int argc, char **argv) {
    // Initialize GoogleTest and run all tests
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}