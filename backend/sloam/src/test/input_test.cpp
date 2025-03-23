#include <gtest/gtest.h>
#include <input.h>

class InputTest : public ::testing::Test {
protected:
    Input input;

    // Queues
    std::deque<StampedSE3> robotOdomQueue_;
    std::queue<Observation> robotObservationQueue_;
    std::deque<RelativeMeas> robotRelativeMeasQueue_;
    std::deque<StampedSE3> robotOdomQueue_1;
    std::queue<Observation> robotObservationQueue_1;
    std::deque<RelativeMeas> robotRelativeMeasQueue_1;
    std::deque<StampedSE3> robotOdomQueue_10;
    std::queue<Observation> robotObservationQueue_10;
    std::deque<RelativeMeas> robotRelativeMeasQueue_10;
    std::deque<StampedSE3> robotOdomQueue_Large;

    // StampedSE3s
    StampedSE3 robotLatestOdom;
    StampedSE3 identity_1;
    StampedSE3 identity_10;
    StampedSE3 oneMeter_1;
    StampedSE3 oneMeter_10;

    InputTest() {
        // Create various ROS times
        ros::Time time_1(1.0);
        ros::Time time_10(10.0);
        ros::Time time_100(100.0);

        // Create StampedSE3s
        identity_1 = StampedSE3(SE3(), time_1);
        identity_10 = StampedSE3(SE3(), time_10);
        SE3 oneMeter = SE3();
        oneMeter.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
        oneMeter_1 = StampedSE3(oneMeter, time_1);
        oneMeter_10 = StampedSE3(oneMeter, time_10);

        // 1 Queues all have stamps of timestep 1
        robotOdomQueue_1.push_back(oneMeter_1);

        Observation obs = Observation();
        obs.stampedPose = identity_1;
        robotObservationQueue_1.push(obs);

        RelativeMeas relMeas = RelativeMeas();
        relMeas.stamp = time_1;
        robotRelativeMeasQueue_1.push_back(relMeas);

        // 10 Queues all have stamps of timestep 10
        robotOdomQueue_10.push_back(oneMeter_10);

        Observation obs_10 = Observation();
        obs_10.stampedPose = identity_10;
        robotObservationQueue_10.push(obs_10);

        RelativeMeas relMeas_10 = RelativeMeas();
        relMeas_10.stamp = time_10;
        robotRelativeMeasQueue_10.push_back(relMeas_10);

        // Add a large number of odometry measurements to the queue
        for (int i = 0; i < 100; i++) {
            robotOdomQueue_Large.push_back(StampedSE3(oneMeter, ros::Time(i)));
        }
    }
};

TEST_F(InputTest, PickNextMeasurementToAdd) {
    // Should return 0 when all queues are empty
    int meas_to_add = 4;
    input.PickNextMeasurementToAdd(robotOdomQueue_, robotObservationQueue_, robotRelativeMeasQueue_, 
                                   robotLatestOdom, 1000.0, 3.0, 0.5, meas_to_add);
    EXPECT_EQ(meas_to_add, 0);

    // Should return 1 when only odom queue has a valid measurement
    input.PickNextMeasurementToAdd(robotOdomQueue_1, robotObservationQueue_, robotRelativeMeasQueue_, 
                                   robotLatestOdom, 1000.0, 3.0, 0.5, meas_to_add);
    EXPECT_EQ(meas_to_add, 1);

    // Should return 2 when only observation queue has a valid measurement
    input.PickNextMeasurementToAdd(robotOdomQueue_, robotObservationQueue_1, robotRelativeMeasQueue_, 
                                   robotLatestOdom, 1000.0, 3.0, 0.5, meas_to_add);
    EXPECT_EQ(meas_to_add, 2);

    // Should return 3 when only relative measurement queue has a valid measurement
    input.PickNextMeasurementToAdd(robotOdomQueue_, robotObservationQueue_, robotRelativeMeasQueue_1, 
                                   robotLatestOdom, 1000.0, 3.0, 0.5, meas_to_add);
    EXPECT_EQ(meas_to_add, 3);

    // Should return queue with the earliest timestamp between Observation and RelativeMeas
    input.PickNextMeasurementToAdd(robotOdomQueue_, robotObservationQueue_1, robotRelativeMeasQueue_10, 
                                   robotLatestOdom, 1000.0, 3.0, 0.5, meas_to_add);
    EXPECT_EQ(meas_to_add, 2);
    input.PickNextMeasurementToAdd(robotOdomQueue_, robotObservationQueue_10, robotRelativeMeasQueue_1, 
                                   robotLatestOdom, 1000.0, 3.0, 0.5, meas_to_add);
    EXPECT_EQ(meas_to_add, 3);

    // Make sure no-longer-useful odometry measurements are popped from the queue
    input.PickNextMeasurementToAdd(robotOdomQueue_Large, robotObservationQueue_, robotRelativeMeasQueue_, 
                                   robotLatestOdom, 76.0, 3.0, 0.5, meas_to_add);
    EXPECT_EQ(meas_to_add, 1);
    EXPECT_EQ(robotOdomQueue_Large.size(), 27);
    EXPECT_EQ(robotOdomQueue_Large.front().stamp.toSec(), 73.0);

    // No measurement should be recommended if the odometry shows the robot hasn't moved enough
    input.PickNextMeasurementToAdd(robotOdomQueue_Large, robotObservationQueue_, robotRelativeMeasQueue_, 
                                   robotLatestOdom, 76.0, 3.0, 1.5, meas_to_add);
    EXPECT_EQ(meas_to_add, 0);
    EXPECT_EQ(robotOdomQueue_Large.size(), 27);
    EXPECT_EQ(robotOdomQueue_Large.front().stamp.toSec(), 73.0);

    // Test the msg_delay_tolerance argument
    input.PickNextMeasurementToAdd(robotOdomQueue_1, robotObservationQueue_10, robotRelativeMeasQueue_10, 
                                   robotLatestOdom, 10.0, 8.0, 0.5, meas_to_add);
    EXPECT_EQ(meas_to_add, 1);
    input.PickNextMeasurementToAdd(robotOdomQueue_1, robotObservationQueue_1, robotRelativeMeasQueue_10, 
                                   robotLatestOdom, 10.0, 8.0, 0.5, meas_to_add);
    EXPECT_EQ(meas_to_add, 2);
    input.PickNextMeasurementToAdd(robotOdomQueue_1, robotObservationQueue_10, robotRelativeMeasQueue_1, 
                                   robotLatestOdom, 10.0, 8.0, 0.5, meas_to_add);
    EXPECT_EQ(meas_to_add, 3);
}

int main(int argc, char **argv) {
    // Run this to allow use of Ros::Time in tests
    ros::Time::init();

    // Initialize GoogleTest and run all tests
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}