//
// Created by Megan Finch on 13/12/2021.
//

#ifndef SFM_CONSTANTS_H
#define SFM_CONSTANTS_H

const float LOWE_RATIO = 0.6f;

// The number of matches required between two images for them to be connected in the scene graph
const int MATCH_COUNT_THRESHOLD = 25;

const double RANSAC_THRESHOLD = 10.0; // RANSAC inlier threshold

const float MAX_REPROJECTION_ERROR = 2000.0f; // Maximum allowed re-projection error in pixels

const bool DEBUG_MODE = true;

const int MIN_POINTS_FOR_HOMOGRAPHY = 10;

const int MIN_POINTS_FOR_FUNDAMENTAL = 15;

const float MIN_RATIO_POSE_INLIERS = 0.15f;

const bool USE_CV_SFM_TRIANGULATION = false;

#endif //SFM_CONSTANTS_H
