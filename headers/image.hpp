#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/core/mat.hpp>
#include <map>

class Image {
    public:
        cv::Mat img;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        // map<keypoint_idx, map<img_idx, keypoint_idx>
        std::map<int, std::map<int, int>> keypoint_matches;
};
#endif