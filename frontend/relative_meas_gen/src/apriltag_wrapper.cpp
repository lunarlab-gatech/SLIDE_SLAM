#include "apriltag_wrapper.h"

apriltag_wrapper::apriltag_wrapper(int id) :
    id (id)
{}

apriltag_wrapper::apriltag_wrapper(int id, double center[2], double corners[4][2], matd_t *rotation, matd_t *translation) :
    id (id)
{
    this->center[0] = center[0];
    this->center[1] = center[1];

    this->rotation = *rotation;
    this->translation = *translation;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            this->corners[i][j] = corners[i][j];
        }
    }
}

apriltag_wrapper::apriltag_wrapper() {}

std::vector<apriltag_wrapper> ExtractAprilTags(cv::Mat img, float intrinsics[4], float tagsize) {

    // Create usable image object from Mat
    image_u8_t img_header = { .width = img.cols,
        .height = img.rows,
        .stride = img.cols,
        .buf = img.data
    };
    image_u8_t * image = &img_header;
    
    // Create apriltag detector object and add desired tag family
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    // Detect apriltags
    zarray_t *detections = apriltag_detector_detect(td, image);

    // Create tag object vector
    std::vector<apriltag_wrapper> tags;

    // Iterate through detected apriltags
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        int id = det->id;
        double center[2];
        double corners[4][2];

        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = tagsize;
        info.fx = intrinsics[0];
        info.cx = intrinsics[1];
        info.fy = intrinsics[2];
        info.cy = intrinsics[3];

        apriltag_pose_t pose;
	    estimate_tag_pose(&info, &pose);

        center[0] = det->c[0];
        center[1] = det->c[1];

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                corners[i][j] = det->p[i][j];
            }
        }

        apriltag_wrapper new_tag(id, center, corners, pose.R, pose.t);
        tags.push_back(new_tag);
    }

    // Free detection objects
    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    return tags;
}

cv::Mat MatFromImage(const sensor_msgs::CompressedImage msg) {
    //Undistort Image
    cv::Mat raw_data(1, msg.data.size(), CV_8UC1, (void*)msg.data.data());
    cv::Mat image = cv::imdecode(raw_data, cv::IMREAD_COLOR);
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}