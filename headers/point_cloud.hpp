//
// Created by Megan Finch on 08/12/2021.
//

#ifndef SFM_POINT_CLOUD_HPP
#define SFM_POINT_CLOUD_HPP

struct StructuredPoint {
    cv::Point3d point;

    // maps image index to keypoint index in that image's keypoints list
    std::map<Image*, int> originatingViews;
};

typedef std::vector<StructuredPoint> PointCloud;

#endif //SFM_POINT_CLOUD_HPP
