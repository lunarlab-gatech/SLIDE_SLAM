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

class slidetag {
    public:
        int id;
        double center[2];
        double corners[4][2];
        matd_t rotation;
	    matd_t translation;

        slidetag(int id, double center[2], double corners[4][2], matd_t *rotation, matd_t *translation);
        slidetag(int id);
        slidetag();
};

std::vector<slidetag> ExtractAprilTags(cv::Mat image, float intrinsics[4]);
cv::Mat MatFromImage(const sensor_msgs::CompressedImage msg);
