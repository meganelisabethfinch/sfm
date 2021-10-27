#include <opencv2/core/mat.hpp>

class Image {
    public:
        cv::Mat img;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
};