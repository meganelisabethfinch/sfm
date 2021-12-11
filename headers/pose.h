//
// Created by Megan Finch on 10/12/2021.
//

#ifndef SFM_POSE_H
#define SFM_POSE_H

struct Pose {
    cv::Mat R; // R as a matrix
    cv::Mat t; // t as a vector
};
#endif //SFM_POSE_H
