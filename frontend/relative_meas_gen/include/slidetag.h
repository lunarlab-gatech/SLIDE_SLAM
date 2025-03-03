#include "apriltag.h"
#include "apriltag_pose.h"
#include "tagStandard52h13.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
#include <map>
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

std::map<int, slidetag> ExtractAprilTags(cv::Mat image);
cv::Mat MatFromImage(std::string path);
