#include "apriltag_meas_gen.h"

void ApriltagMeasurer::imageCallback(const sensor_msgs::CompressedImage msg) {
      
    // Extract image
    cv::Mat img = MatFromImage(msg);
    if (img.empty()) {
        std::cerr << "Error: Decoded image is empty!" << std::endl;
        return;
    }

    std::vector<slidetag> tags = ExtractAprilTags(img, this->intrinsics, this->tagsize);\

    // Publish transformation for each detected apriltag
    for (slidetag t : tags) {
        int8_t bot_id;
        Eigen::Matrix4f bot_to_cam;
        Eigen::Matrix4f cam_to_tag;
        Eigen::Matrix4f tag_to_bot;

        // Obtain RT from camera -> detected apriltag
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cam_to_tag(i, j) = t.rotation.data[i + j];
            }
            cam_to_tag(3, i) = t.translation.data[i];
        }
        std::cout << "Tag ID: " << t.id << std::endl;

        std::tuple<int8_t, std::array<Eigen::Matrix4f, 2>> loaded_transformations = LoadTransformations(t);
        std::array<Eigen::Matrix4f, 2> transformations = std::get<1>(loaded_transformations);
        bot_id = std::get<0>(loaded_transformations);
        
        if (bot_id == -1) {
            ROS_ERROR("Transformations not loaded for apriltag id: " + t.id);
            continue;
        }
        bot_to_cam = transformations[0];
        tag_to_bot = transformations[1];

        Eigen::Matrix4f total_transformation = CalculateRelativeTransformation(bot_to_cam, cam_to_tag, tag_to_bot);
        PublishRelativeMeasurement(bot_id, total_transformation);
    }

}

std::tuple<int8_t, std::array<Eigen::Matrix4f, 2>> ApriltagMeasurer::LoadTransformations(slidetag t) {
    std::tuple<int8_t, std::array<Eigen::Matrix4f, 2>> return_tuple;
    int8_t bot_id = -1;
    std::array<Eigen::Matrix4f, 2> transformations;

    std::string dataset = config["dataset"].as<std::string>();
    if (dataset == "CoPeD") {
        // Load RT matrix from host bot -> camera from config
        Eigen::Matrix4f bot_to_cam = Eigen::Matrix4f::Identity();
        if (config[robot_ID][camera_ID]["R"].IsDefined()) {
            YAML::Node R = config[robot_ID][camera_ID]["R"];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    bot_to_cam(i, j) = R[i][j].as<float>();
                }
            }
            bot_to_cam(0, 3) = config[robot_ID][camera_ID]["x"].as<float>();
            bot_to_cam(1, 3) = config[robot_ID][camera_ID]["y"].as<float>();
            bot_to_cam(2, 3) = config[robot_ID][camera_ID]["z"].as<float>();
        } else {
            float x = config[robot_ID][camera_ID]["x"].as<float>();
            float y = config[robot_ID][camera_ID]["y"].as<float>();
            float z = config[robot_ID][camera_ID]["z"].as<float>();
            float roll = config[robot_ID][camera_ID]["roll"].as<float>();
            float pitch = config[robot_ID][camera_ID]["pitch"].as<float>();
            float yaw = config[robot_ID][camera_ID]["yaw"].as<float>();
            bot_to_cam = RollPitchYaw_to_RT(x, y, z, roll, pitch, yaw);
        }

        Eigen::Matrix4f tag_to_bot;
        bool tag_on_bot = false;

        // Obtain translation vector and quaternion from detected apriltag -> detected robot
        for (auto it = config.begin(); it != config.end(); ++it) {
            std::string key = it->first.as<std::string>();
            if (config[key]["tags"].IsDefined()) {
                YAML::Node tags = config[key]["tags"];
                for (auto tag = tags.begin(); tag != tags.end(); ++tag) {
                    if (t.id == (*tag)["id"].as<int>()) {
                        bot_id = config[key]["id"].as<int>();
                        float x = (*tag)["x"].as<float>();
                        float y = (*tag)["y"].as<float>();
                        float z = (*tag)["z"].as<float>();
                        Eigen::Vector3f translation = Eigen::Vector3f(x, y, z);
                        float qw = (*tag)["qw"].as<float>();
                        float qx = (*tag)["qx"].as<float>();
                        float qy = (*tag)["qy"].as<float>();
                        float qz = (*tag)["qz"].as<float>();
                        Eigen::Quaternionf q = Eigen::Quaternionf(qw, qx, qy, qz);
                        tag_on_bot = true;
                        tag_to_bot = Eigen::Matrix4f::Identity();
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

        transformations[0] = bot_to_cam;
        transformations[1] = tag_to_bot;
        return_tuple = std::make_tuple(bot_id, transformations);
        return return_tuple;
        
    } else {
        ROS_INFO("Loading from config for requested dataset not supported.");
    }
}

ApriltagMeasurer::ApriltagMeasurer(ros::NodeHandle nh): nh_(nh) {
    
    // Load params
    std::string image_topic;
    nh_.param<std::string>("apriltag_node/image_topic", image_topic, "/default_topic");
    nh_.param<std::string>("apriltag_node/host_robot", robot_ID, "robot0");
    nh_.param<float>("apriltag_node", tagsize, .17);
    nh_.param<std::string>("apriltag_node/camera_ID", camera_ID, "camera0");
    std::cout << "Subscribing to topic: " << image_topic << std::endl;
    std::string config_file;
    nh_.param<std::string>("apriltag_node/config_file", config_file, "default.yaml");
    std::string return_topic;
    nh_.param<std::string>("apriltag_node/relative_meas", return_topic, "/default_topic");

    // Load config file
    config = YAML::LoadFile(config_file);

    // Load host_robot properties
    float fx = config[robot_ID][camera_ID]["fx"].as<float>();
    float cx = config[robot_ID][camera_ID]["cx"].as<float>();
    float fy = config[robot_ID][camera_ID]["fy"].as<float>();
    float cy = config[robot_ID][camera_ID]["cy"].as<float>();

    intrinsics[0] = fx;
    intrinsics[1] = cx;
    intrinsics[2] = fy;
    intrinsics[3] = cy;

    // Instantiate sub and pub
    robot_images = nh_.subscribe(image_topic, 1, &ApriltagMeasurer::imageCallback, this);
    relative_meas = nh_.advertise<geometry_msgs::Pose>(return_topic, 10);
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

void ApriltagMeasurer::PublishRelativeMeasurement(int8_t bot_id, Eigen::Matrix4f transformation) {
    
    geometry_msgs::Pose pose_msg;
    sloam_msgs::RelativeInterRobotMeasurement msg;

    geometry_msgs::Point position;
    geometry_msgs::Quaternion orientation;

    /*
    TODO: some math to get these two fields
    */

    msg.header.stamp = ros::Time::now();
    msg.relativePose = pose_msg;
    
    /*
    TODO: Get odometry data
    */

    msg.robotId = bot_id;

    std::cout << "Publishing pose from host to " << bot_id << std::endl;

    relative_meas.publish(pose_msg);
}