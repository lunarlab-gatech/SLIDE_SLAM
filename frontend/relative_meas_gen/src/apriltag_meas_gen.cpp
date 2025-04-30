#include "apriltag_meas_gen.h"

void ApriltagMeasurer::imageCallback(const sensor_msgs::CompressedImage msg) {
      
    // Extract image
    cv::Mat img = MatFromImage(msg, this->intrinsics, this->dist_coefficients);
    if (img.empty()) {
        std::cerr << "Error: Decoded image is empty!" << std::endl;
        return;
    }

    ros::Time timestamp = msg.header.stamp;

    std::vector<apriltag_wrapper> tags = ExtractAprilTags(img, this->intrinsics, this->tagsize);\

    // Publish transformation for each detected apriltag
    for (apriltag_wrapper t : tags) {
        int8_t bot_id;
        Eigen::Matrix4d cam_to_tag;
        Eigen::Matrix4d tag_to_bot;

        // Obtain RT from camera -> detected apriltag
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cam_to_tag(i, j) = t.rotation.data[i + j];
            }
            cam_to_tag(i, 3) = t.translation.data[i];
            cam_to_tag(3, i) = 0.0;
        }
        cam_to_tag(3, 3) = 1.0;
        std::cout << "Tag ID: " << t.id << std::endl;

        std::tuple<int8_t, Eigen::Matrix4d> loaded_transformations = LoadTransformations(t);
        tag_to_bot = std::get<1>(loaded_transformations);
        bot_id = std::get<0>(loaded_transformations);
        
        if (bot_id == -1) {
            ROS_ERROR("Transformations not loaded for apriltag");
            continue;
        }

        Eigen::Matrix4d total_transformation = CalculateRelativeTransformation(bot_to_cam, cam_to_tag, tag_to_bot);
        PublishRelativeMeasurement(bot_id, total_transformation, timestamp);
    }

}

std::tuple<int8_t, Eigen::Matrix4d> ApriltagMeasurer::LoadTransformations(apriltag_wrapper t) {
    std::tuple<int8_t, Eigen::Matrix4d> return_tuple;
    int8_t bot_id;
    Eigen::Matrix4d tag_to_bot;

    std::string dataset = config["dataset"].as<std::string>();
    if (dataset == "CoPeD") {

        bool tag_on_bot = false;

        // Obtain translation vector and quaternion from detected apriltag -> detected robot
        for (auto it = ++config.begin(); it != config.end(); ++it) {
            std::string key = it->first.as<std::string>();
            if (config[key]["tags"].IsDefined()) {
                YAML::Node tags = config[key]["tags"];
                for (auto tag = tags.begin(); tag != tags.end(); ++tag) {
                    if (t.id == (*tag)["id"].as<int>()) {
                        bot_id = config[key]["id"].as<int8_t>();
                        double x = (*tag)["x"].as<double>();
                        double y = (*tag)["y"].as<double>();
                        double z = (*tag)["z"].as<double>();
                        Eigen::Vector3d translation = Eigen::Vector3d(x, y, z);
                        double qw = (*tag)["qw"].as<double>();
                        double qx = (*tag)["qx"].as<double>();
                        double qy = (*tag)["qy"].as<double>();
                        double qz = (*tag)["qz"].as<double>();
                        Eigen::Quaterniond q = Eigen::Quaterniond(qw, qx, qy, qz);
                        tag_on_bot = true;
                        tag_to_bot = Eigen::Matrix4d::Identity();
                        tag_to_bot.block<3,3>(0,0) = q.toRotationMatrix();
                        tag_to_bot.block<3,1>(0,3) = translation;
                        break;
                    }
                }
            }
        }
        if (!tag_on_bot) {
            ROS_WARN("Detected tag does not belong to any robot in dataset");
        }
        return_tuple = std::make_tuple(bot_id, tag_to_bot);
        return return_tuple;
        
    } else {
        ROS_INFO("Loading from config for requested dataset not supported.");
    }
    return std::make_tuple(-1, Eigen::Matrix4d::Identity());;
}

ApriltagMeasurer::ApriltagMeasurer(ros::NodeHandle nh): nh_(nh) {
    
    // Load params
    std::string image_topic;
    nh_.param<std::string>("apriltag_node/image_topic", image_topic, "/default_topic");
    std::string host_robot;
    nh_.param<std::string>("apriltag_node/host_robot", host_robot, "robot0");
    nh_.param<int>("apriltag_node/robot_ID", robot_ID, 0);
    nh_.param<double>("apriltag_node", tagsize, .17);
    nh_.param<std::string>("apriltag_node/camera_ID", camera_ID, "camera0");
    std::string config_file;
    nh_.param<std::string>("apriltag_node/config_file", config_file, "default.yaml");
    std::string return_topic;
    nh_.param<std::string>("apriltag_node/relative_meas", return_topic, "/default_topic");

    // Load config file
    config = YAML::LoadFile(config_file);

    std::string dataset = config["dataset"].as<std::string>();
    if (dataset == "CoPeD") {
        if (robot_ID == 0 || robot_ID ==1) {
            std::string base_link;
            std::string camera_link;
            nh_.param<std::string>("apriltag_node/base_link", base_link, "/default_base_link");
            nh_.param<std::string>("apriltag_node/camera_link", camera_link, "/default_cam_link");

            // Get bot to cam transformation from tf tree
            tf::TransformListener listener;
            tf::StampedTransform transformStamped;

            listener.waitForTransform(camera_link, base_link, ros::Time::now(), ros::Duration(5.0));
            listener.lookupTransform(camera_link, base_link, ros::Time::now(), transformStamped);

            double x, y, z, qw, qx, qy, qz;
            x = transformStamped.getOrigin().x();
            y = transformStamped.getOrigin().y();
            z = transformStamped.getOrigin().z();
            qw = transformStamped.getRotation().w();
            qx = transformStamped.getRotation().x();
            qy = transformStamped.getRotation().y();
            qz = transformStamped.getRotation().z();

            Eigen::Vector3d translation = Eigen::Vector3d(x, y, z);
            Eigen::Quaterniond q = Eigen::Quaterniond(qw, qx, qy, qz);

            bot_to_cam = Eigen::Matrix4d::Identity();
            bot_to_cam.block<3,1>(0,3) = translation;
            bot_to_cam.block<3,3>(0,0) = q.toRotationMatrix();
        } else {
            bot_to_cam = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    bot_to_cam(i, j) = config[host_robot][camera_ID]["T_cam_imu"][i][j].as<double>();
                }
            }
        }
    }

    // Load host_robot properties
    double fx = config[host_robot][camera_ID]["fx"].as<double>();
    double cx = config[host_robot][camera_ID]["cx"].as<double>();
    double fy = config[host_robot][camera_ID]["fy"].as<double>();
    double cy = config[host_robot][camera_ID]["cy"].as<double>();

    intrinsics[0] = fx;
    intrinsics[1] = cx;
    intrinsics[2] = fy;
    intrinsics[3] = cy;

    for (int i = 0; i < 4; i++) {
        dist_coefficients[i] = config[host_robot][camera_ID]["distortion_coeffs"][i].as<double>();
    }

    // Instantiate sub and pub
    robot_images_sub = nh_.subscribe(image_topic, 1, &ApriltagMeasurer::imageCallback, this);
    relative_meas_pub = nh_.advertise<sloam_msgs::RelativeInterRobotMeasurement>(return_topic, 10);
    std::cout << "Subscribed to topic: " << image_topic << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_node");
    ros::NodeHandle nh;
    ApriltagMeasurer measurer(nh);
    ros::spin();
    return 0;
}

/*
 * This function calculates the relative transformation between
 * two robots.
 * 
 * Parameters:
 *  H_hostBot_to_cam - Transformation from the host robot base frame 
 *                  to the camera frame
 *  H_cam_to_tag - Transformation from the camera to the observed tag
 *  H_observedBot_to_tag - Transformation from the observed robot base 
 *                  frame to the observed tag
 * 
 * Returns:
 *  Eigen::Matrix4d -  Transformation from the host robot base frame to 
 *                  the observed robot base frame.
 */
Eigen::Matrix4d ApriltagMeasurer::CalculateRelativeTransformation(Eigen::Matrix4d H_hostBot_to_cam, 
                                    Eigen::Matrix4d H_cam_to_tag, Eigen::Matrix4d H_observedBot_to_tag) {
    
    // Calculate transformation from bot_to_tag
    Eigen::Matrix4d H_bot_to_tag = H_hostBot_to_cam * H_cam_to_tag;
    
    // Invert to get tag to observedBot
    Eigen::Matrix4d T_tag_to_observedBot = H_observedBot_to_tag.inverse();

    // Chain to get bot -> observedBot
    Eigen::Matrix4d T_bot_to_observedBot = H_bot_to_tag * T_tag_to_observedBot;
    return T_bot_to_observedBot;
}

void ApriltagMeasurer::PublishRelativeMeasurement(int8_t bot_id, Eigen::Matrix4d transformation, ros::Time timestamp) {
    
    geometry_msgs::Pose pose_msg;
    sloam_msgs::RelativeInterRobotMeasurement msg;

    geometry_msgs::Point position;
    geometry_msgs::Quaternion orientation;

    position.x = (float) transformation(0, 3);
    position.y = (float) transformation(1, 3);
    position.z = (float) transformation(2, 3);

    Eigen::Matrix3d rotation = bot_to_cam.block<3,3>(0,0);
    Eigen::Quaterniond quat(rotation);

    orientation.x = (float) quat.x();
    orientation.y = (float) quat.y();
    orientation.z = (float) quat.z();
    orientation.w = (float) quat.w();

    pose_msg.position = position;
    pose_msg.orientation = orientation;

    msg.header.stamp = timestamp;
    msg.relativePose = pose_msg;
    msg.robotIdObserved = bot_id;
    msg.robotIdObserver = robot_ID;

    double mag = pow(pow(position.x, 2) + pow(position.y, 2) + pow(position.z, 2), .5);
    
    std::cout << "Transformation magnitude: " << mag << std::endl;
    std::cout << "Publishing pose from robot " << robot_ID << " to robot " << bot_id << std::endl;

    relative_meas_pub.publish(msg);
}