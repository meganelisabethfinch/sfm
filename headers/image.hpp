#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/core/mat.hpp>
#include <map>

class Image {
    public:
        std::string name;
        cv::Mat img;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        // map<imgIdx, map<keypointIdx, keypointIdx>>
        // the first keypoint index is an index into this.keypoints
        // the second keypoint index is an index into image_j.keypoints (?)
        std::map<int, std::map<int, int>> keypoint_matches;
};
#endif