#!/usr/bin/env python

import unittest
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from relative_meas_gen.transforms import calculate_relative_pose

class TestMultiUGV(unittest.TestCase):
    """
    Test multiUGVToSlideSLAM.py
    """

    def setUp(self):
        rospy.init_node('multi_ugv_test_node', anonymous=True)

    def test_calculate_relative_pose(self):
        """
        Test that the calculate_relative_pose method properly calculates
        the relevant transform between two poses.
        """

        # Setup two test poses
        pose1 = Pose(position=Pose().position.__class__(3.4, -5.2, 1.1), orientation=Pose().orientation.__class__(0.4029115, 0.1611646, 0.805823, 0.4029115))
        pose2 = Pose(position=Pose().position.__class__(1, 2, 3), orientation=Pose().orientation.__class__(0, 0, 0.7071068, 0.7071068))

        # Call the method
        rel_pose = calculate_relative_pose(pose1, pose2)

        # Make sure it's correct
        np.testing.assert_almost_equal(rel_pose.position.x, 7.43896085266152, 6)
        np.testing.assert_almost_equal(rel_pose.position.y, -2.13116887703829, 6)
        np.testing.assert_almost_equal(rel_pose.position.z, -1.15324631249453, 6)
        np.testing.assert_almost_equal(rel_pose.orientation.x, -0.398862)
        np.testing.assert_almost_equal(rel_pose.orientation.y, 0.1709409)
        np.testing.assert_almost_equal(rel_pose.orientation.z, -0.2849014)
        np.testing.assert_almost_equal(rel_pose.orientation.w, 0.8547043)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('relative_meas_gen', 'test_multi_ugv', TestMultiUGV)
    rostest.rosrun('relative_meas_gen', 'apriltag_test', 'Detect')