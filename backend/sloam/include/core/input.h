#include <robot.h>

class Input {

public:
    void PickNextMeasurementToAdd(std::deque<StampedSE3> &robotOdomQueue_, 
                                  std::queue<Observation> &robotObservationQueue_,
                                  std::deque<RelativeMeas> &robotRelativeMeasQueue_,
                                  StampedSE3 &robotLatestOdom_, 
                                  double current_time,
                                  double msg_delay_tolerance,
                                  float minOdomDistance_,
                                  int &meas_to_add);
};