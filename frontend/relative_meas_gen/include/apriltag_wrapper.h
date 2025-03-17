#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <sensor_msgs/CompressedImage.h>
#include <vector>
#include <string>

class apriltag_wrapper {
    public:
        int id;
        double center[2];
        double corners[4][2];
        matd_t rotation;
	    matd_t translation;

        apriltag_wrapper(int id, double center[2], double corners[4][2], matd_t *rotation, matd_t *translation);
        apriltag_wrapper(int id);
        apriltag_wrapper();
};

std::vector<apriltag_wrapper> ExtractAprilTags(cv::Mat image, float intrinsics[4], float tagsize);
cv::Mat MatFromImage(const sensor_msgs::CompressedImage msg);
