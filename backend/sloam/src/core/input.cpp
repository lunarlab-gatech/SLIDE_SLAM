#include <input.h>

/**
 * @brief This measurement handles logic for picking which measurement to add
 * to the factor graph next. It ensures they are added in order of timestamp 
 * (assuming that each queue is ordered by timestamp). This prevents jumping
 * back and forth through time in the factor graph, which breaks covariances.
 * 
 * @param robotOdomQueue_: The deque of odometry measurements.
 * @param robotObservationQueue_: The queue of observation measurements.
 * @param robotRelativeMeasQueue_: The deque of relative measurements.
 * @param current_time: The current time of the robot (in seconds)
 * @param msg_delay_tolerance: The amount of time to wait before adding
 *                    a measurement.
 * @param minOdomDistance_: The minimum distance of estimated odometry 
 *                    movement to add a new odometry measurement.  
 * @param meas_to_add: Value will represent which measurement to add.
 *                    0: None
 *                    1: Odometry
 *                    2: Observation
 *                    3: Relative Measurement
 */
void Input::PickNextMeasurementToAdd(std::deque<StampedSE3> &robotOdomQueue_, 
                                     std::queue<Observation> &robotObservationQueue_,
                                     std::deque<RelativeMeas> &robotRelativeMeasQueue_, 
                                     StampedSE3 &robotLatestOdom_,
                                     double current_time,
                                     double msg_delay_tolerance,
                                     float minOdomDistance_,
                                     int &meas_to_add) {

  // Create variables to hold statuses and values of valid measurements
  bool validObs = false;
  bool validRelMeas = false;
  Observation obs;
  RelativeMeas relMeas;

  // Get the oldest observation if it exists
  if (!robotObservationQueue_.empty()) {
    obs = robotObservationQueue_.front();
    validObs = (current_time - obs.stampedPose.stamp.toSec()) >= msg_delay_tolerance;
  } else {
    validObs = false;
  }

  // Get the oldest relative measurement if it exists
  if (!robotRelativeMeasQueue_.empty()) {
    relMeas = robotRelativeMeasQueue_.front();
    validRelMeas = (current_time - relMeas.stamp.toSec()) >= msg_delay_tolerance;
  } else {
    validRelMeas = false;
  }

  // If they are both valid, add the oldest one
  if (validObs && validRelMeas) {
    if (obs.stampedPose.stamp < relMeas.stamp) { meas_to_add = 2; } 
    else { meas_to_add = 3; }
    return;
  } 
  
  // If only one is valid, add that
  else if (validObs || validRelMeas) {
    if (validObs)  {  meas_to_add = 2; }
    else if (validRelMeas) { meas_to_add = 3; }
    return;
  }

  // If neither are valid, check if we have a valid odometry measurement
  for (int i = robotOdomQueue_.size() - 1; i >= 0; i--) {

    // Check if this odometry measurement is valid yet. Otherwise, move to next one
    if ((current_time - robotOdomQueue_[i].stamp.toSec()) >= msg_delay_tolerance) {

      // If it is, check if we have moved enough to add a new odometry measurement
      SE3 currRelativeMotion = robotLatestOdom_.pose.inverse() * robotOdomQueue_[i].pose;
      double accumMovement = currRelativeMotion.translation().norm();
      if (accumMovement > minOdomDistance_) {

        // Add the odometry measurement
        meas_to_add = 1;

        // Pop all previous odometry measurements, as we won't add them
        for (int j = 0; j < i; j++) {
          robotOdomQueue_.pop_front();
        }
        return;
      }
      break;
    }
  }
  
  // If we got this far, then no odometry measurements are valid & moved enough
  meas_to_add = 0;
  return;
}

/**
 * @brief Computes the relative motion covariance from odometry pose T1 to odometry
 *       pose T2 given their respective covariance matrices. Note that cov2 includes 
 *       the covariance of T1 plus any additional covariance from the relative motion.
 * 
 * @param T1: The first odometry pose
 * @param cov1: The associated covariance for T1
 * @param T2: The second odometry pose
 * @param cov2: The associated covariance for T2
 * @return The covariance for the relative motion
 */
// std::array<double, 6> Input::computeRelativeMotionCovariance(const SE3 &T1, 
//   const std::array<double, 6> &cov1, const SE3 &T2, const std::array<double, 6> &cov2) {

//   std::array<double, 6> cov_rel;

//   // Compute relative transformation
//   SE3 T_rel = T1.inverse() * T2;

//   // Extract rotation and translation from T_rel
//   Eigen::Matrix3d R_rel = T_rel.rotationMatrix();
//   Eigen::Vector3d t_rel = T_rel.translation();

//   // Compute Adjoint matrix
//   Eigen::Matrix<double, 6, 6> Adj_T_rel = Eigen::Matrix<double, 6, 6>::Zero();
//   Adj_T_rel.block<3, 3>(0, 0) = R_rel;
//   Adj_T_rel.block<3, 3>(0, 3) = skewSymmetric(t_rel) * R_rel;
//   Adj_T_rel.block<3, 3>(3, 3) = R_rel;

//   // Convert std::array to Eigen::Matrix
//   Eigen::Matrix<double, 6, 6> Sigma1 = Eigen::Matrix<double, 6, 6>::Zero();
//   Eigen::Matrix<double, 6, 6> Sigma2 = Eigen::Matrix<double, 6, 6>::Zero();

//   for (int i = 0; i < 6; ++i) {
//       Sigma1(i, i) = cov1[i];
//       Sigma2(i, i) = cov2[i];
//   }

//   // Compute the relative motion covariance
//   Eigen::Matrix<double, 6, 6> Sigma1_transformed = Adj_T_rel * Sigma1 * Adj_T_rel.transpose();
//   Eigen::Matrix<double, 6, 6> Sigma_rel = Sigma2 - Sigma1_transformed;

//   // Store result back in std::array
//   for (int i = 0; i < 6; ++i) {
//       cov_rel[i] = Sigma_rel(i, i);  // Extract diagonal elements
//   }

//   return cov_rel;
// }