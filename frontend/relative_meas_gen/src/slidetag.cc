#include "slidetag.h"

slidetag::slidetag(int id) :
    id (id)
{}

slidetag::slidetag(int id, double center[2], double corners[4][2]) :
    id (id)
{
    this->center[0] = center[0];
    this->center[1] = center[1];

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            this->corners[i][j] = corners[i][j];
        }
    }
}

slidetag::slidetag() {}

std::map<int, slidetag> ExtractAprilTags(cv::Mat img, float fx, float fy, float cx, float cy, float tag_size) {

    // Create usable image object from Mat
    image_u8_t img_header = { .width = img.cols,
        .height = img.rows,
        .stride = img.cols,
        .buf = img.data
    };

    image_u8_t * image = &img_header;
    
    // Create apriltag detector object and add desired tag family
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tagStandard52h13_create();
    apriltag_detector_add_family(td, tf);

    // Detect apriltags
    zarray_t *detections = apriltag_detector_detect(td, image);

    // Create tag object vector
    std::map<int, slidetag> tags;

    // Iterate through detected apriltags
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        int id = det->id;
        double center[2];
        double corners[4][2];

        center[0] = det->c[0];
        center[1] = det->c[1];

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                corners[i][j] = det->p[i][j];
            }
        }

        slidetag new_tag(id, center, corners);
        tags[id] = new_tag;
    }

    // Free detection objects
    apriltag_detections_destroy(detections);
    tagStandard52h13_destroy(tf);
    apriltag_detector_destroy(td);

    return tags;
}

cv::Mat MatFromImage(std::string path) {
    //Undistort Image
    cv::Mat matrix = cv::imread(path, cv::IMREAD_COLOR).clone();
    cv::Mat gray;
    cv::cvtColor(matrix, gray, cv::COLOR_BGR2GRAY);
    return gray;
}