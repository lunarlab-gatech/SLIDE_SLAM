#include <input.h>

/**
 * @brief This measurement handles logic for picking which measurement to add
 * to the factor graph next. It ensures they are added in order of timestamp 
 * (assuming that each queue is ordered by timestamp). This prevents jumping
 * back and forth through time in the factor graph, which breaks covariances.
 * To do this, it will pop measurements that are before the most recent factor.
 * 
 * @param robotOdomQueue_: The deque of odometry measurements.
 * @param robotObservationQueue_: The queue of observation measurements.
 * @param robotRelativeMeasQueue_: The deque of relative measurements.
 * @param robotLatestOdom_: The latest estimated pose from odometry used to
 *        generate a relativeRawOdomMotion for the factor graph.
 * @param current_time: The current time of the robot (in seconds)
 * @param msg_delay_tolerance: The amount of time to wait before adding
 *                    a measurement.
 * @param minOdomDistance_: The minimum distance of estimated odometry 
 *                    movement to add a new odometry measurement.  
 * @param[out] meas_to_add: Value will represent which measurement to add.
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

  // Pop all measurements that are before the most recent graph factor
  while (!robotOdomQueue_.empty() && robotOdomQueue_.front().stamp < robotLatestOdom_.stamp) {
    robotOdomQueue_.pop_front();
  }
  while (!robotObservationQueue_.empty() && robotObservationQueue_.front().stampedPose.stamp < robotLatestOdom_.stamp) {
    robotObservationQueue_.pop();
  }
  while (!robotRelativeMeasQueue_.empty() && robotRelativeMeasQueue_.front().stamp < robotLatestOdom_.stamp) {
    robotRelativeMeasQueue_.pop_front();
  }

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