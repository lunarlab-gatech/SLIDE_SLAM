#include "apriltag_meas_gen.h"

void ApriltagMeasurer::imageCallback(const sensor_msgs::CompressedImage msg) {
      
    cv::Mat img = MatFromImage(msg);
    if (img.empty()) {
        std::cerr << "Error: Decoded image is empty!" << std::endl;
        return;
    }

    std::vector<slidetag> tags = ExtractAprilTags(img, this->intrinsics);\

    Eigen::Matrix4f bot_cam_RT;
    YAML::Node R = config[robot_ID][camera_ID]["R"];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            bot_cam_RT(i, j) = R[i][j].as<float>();
        }
    }
    bot_cam_RT(0, 3) = config[robot_ID][camera_ID]["x"].as<float>();
    bot_cam_RT(1, 3) = config[robot_ID][camera_ID]["y"].as<float>();
    bot_cam_RT(2, 3) = config[robot_ID][camera_ID]["z"].as<float>();

    for (slidetag t : tags) {
        Eigen::Matrix4f cam_tag_RT;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cam_tag_RT(i, j) = t.rotation.data[i + j];
            }
            cam_tag_RT(3, i) = t.translation.data[i];
        }
        std::cout << "Tag ID: " << t.id << std::endl;

        float x;
        float y;
        float z;
        float qw;
        float qx;
        float qy;
        float qz;
        bool tag_on_bot = false;
        for (auto it = config.begin(); it != config.end(); ++it) {
            std::string key = it->first.as<std::string>();
            if (config[key]["tags"].IsDefined()) {
                YAML::Node tags = config[key]["tags"];
                for (auto tag = tags.begin(); tag != tags.end(); ++tag) {
                    if (t.id == (*tag)["id"].as<int>()) {
                        x = (*tag)["x"].as<float>();
                        y = (*tag)["y"].as<float>();
                        z = (*tag)["z"].as<float>();
                        qw = (*tag)["qw"].as<float>();
                        qx = (*tag)["qx"].as<float>();
                        qy = (*tag)["qy"].as<float>();
                        qz = (*tag)["qz"].as<float>();
                        tag_on_bot = true;
                        break;
                    }
                }
            }
        }
        if (tag_on_bot) {
            std::cout << "Detected tag does not belong to any robot" << std::endl;
        } else {
            Eigen::Vector3f bot_tag_T;
            bot_tag_T << x, y, z;
            Eigen::Quaternionf bot_tag_Q(qw, qx, qy, qz);

            publishRelativeMeasurement(bot_cam_RT, cam_tag_RT, bot_tag_T, bot_tag_Q);

        }
    }

}

ApriltagMeasurer::ApriltagMeasurer(ros::NodeHandle nh): nh_(nh) {
    std::string image_topic;
    nh_.param<std::string>("apriltag_node/image_topic", image_topic, "/default_topic");

    nh_.param<std::string>("apriltag_node/host_robot", robot_ID, "robot0");
    
    nh_.param<std::string>("apriltag_node/camera_ID", camera_ID, "camera0");
    std::cout << "Subscribing to topic: " << image_topic << std::endl;
    std::string config_file;
    nh_.param<std::string>("apriltag_node/config_file", config_file, "default.yaml");
    std::string return_topic;
    nh_.param<std::string>("apriltag_node/relative_meas", return_topic, "/default_topic");

    config = YAML::LoadFile(config_file);

    float fx = config[robot_ID][camera_ID]["fx"].as<float>();
    float cx = config[robot_ID][camera_ID]["cx"].as<float>();
    float fy = config[robot_ID][camera_ID]["fy"].as<float>();
    float cy = config[robot_ID][camera_ID]["cy"].as<float>();

    intrinsics[0] = fx;
    intrinsics[1] = cx;
    intrinsics[2] = fy;
    intrinsics[3] = cy;

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

void ApriltagMeasurer::publishRelativeMeasurement(Eigen::Matrix4f bot_to_cam_RT, Eigen::Matrix4f cam_to_tag_RT, Eigen::Vector3f bot_to_tag_T, Eigen::Quaternionf bot_to_tag_Q) {
    
    geometry_msgs::Pose pose_msg;

    /*
    Calculate pose
    */

    std::cout << "Publishing pose" << std::endl;

    relative_meas.publish(pose_msg);
}