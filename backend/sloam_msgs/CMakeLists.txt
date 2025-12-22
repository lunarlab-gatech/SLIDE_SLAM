cmake_minimum_required(VERSION 3.16)
project(sloam_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib_msgs
  visualization_msgs
)

add_message_files(DIRECTORY msg FILES
    interRobotTF.msg
    ROSCylinder.msg
    ROSEllipsoid.msg
    ROSGround.msg
    ROSScan.msg
    ROSSweep.msg
    ROSSubMap.msg
    ROSObservation.msg
    ObservationPair.msg
    OdometryGPSBundle.msg
    LoopClosure.msg
    keyPoses.msg
    cubeMap.msg
    cylinderMap.msg
    MultiArrayPoseObjectEdges.msg
    poseObjectEdges.msg 
    RelativeInterRobotMeasurement.msg
    RelativeInterRobotMeasurementOdom.msg
    ROSCube.msg
    PoseMst.msg
    PoseMstBundle.msg
    ROSRangeBearing.msg
    ROSRangeBearingSyncOdom.msg
    SemanticLoopClosure.msg
    ROSSyncOdom.msg
    ROSCylinderArray.msg
    SemanticMeasSyncOdom.msg
    StampedRvizMarkerArray.msg
    vector4d.msg
    vector7d.msg
    syncPcOdom.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  ActiveLoopClosure.action
  DetectLoopClosure.action
)

add_service_files(
  FILES
  EvaluateLoopClosure.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs actionlib_msgs visualization_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs nav_msgs actionlib_msgs visualization_msgs
)
