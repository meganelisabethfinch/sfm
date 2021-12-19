//
// Created by Megan Finch on 13/12/2021.
//

#ifndef SFM_CONSTANTS_H
#define SFM_CONSTANTS_H

const float LOWE_RATIO = 0.6f;

// The number of matches required between two images for them to be connected in the scene graph
const int MATCH_COUNT_THRESHOLD = 25;

const double RANSAC_THRESHOLD = 10.0; // RANSAC inlier threshold

const float MAX_REPROJECTION_ERROR = 10.0f; // Maximum 10-pixel allowed re-projection error

const bool DEBUG_MODE = true;

const bool USE_CV_SFM_TRIANGULATION = false;

const cv::Mat I = cv::Mat::eye(3,3,CV_32F);

typedef int ImageID;
typedef int KeyPointIDX;

#endif //SFM_CONSTANTS_H
