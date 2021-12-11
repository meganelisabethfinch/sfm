#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/core/mat.hpp>
#include <map>
#include "pose.h"

class Image {
    public:
        std::string name;
        cv::Mat img;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        // map<imgIdx, map<keypointIdx, keypointIdx>>
        // the first keypoint index is an index into this.keypoints
        // the second keypoint index is an index into image_j.keypoints (?)
        std::map<Image*, std::map<int, int>> keypoint_matches;

        // map<imgIdx, fundamentalMatrix&>
        std::map<Image*, cv::Mat> fundamentalMatrices;

        Pose pose;
};
#endif