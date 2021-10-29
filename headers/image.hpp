#include <opencv2/core/mat.hpp>

#ifndef IMAGE_H
#define IMAGE_H

class Image {
    public:
        cv::Mat img;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
};
#endif