//
// Created by Megan Finch on 19/12/2021.
//

#include "sfm_utilities.hpp"

#include <image.hpp>
#include <opencv2/calib3d.hpp>


int findHomographyInliers(Image &image1, Image &image2) {
    std::vector<cv::DMatch> matches = image1.getMatchesByImage(image2.getId());

    std::vector<cv::Point2f> alignedLeft = image1.getMatchedPoints(image2.getId());
    std::vector<cv::Point2f> alignedRight = image2.getMatchedPoints(image1.getId());

    cv::Mat inlierMask;
    cv::Mat H;

    if (matches.size() >= 4) {
        H = cv::findHomography(alignedLeft,
                               alignedRight,
                               cv::RANSAC,
                               RANSAC_THRESHOLD,
                               inlierMask);
    }

    if (matches.size() < 4 || H.empty()) {
        return 0;
    }

    return countNonZero(inlierMask);
}

void getPointsFromMatches(std::vector<cv::KeyPoint>& leftKeyPoints,
                          std::vector<cv::KeyPoint>& rightKeyPoints,
                          std::vector<cv::DMatch>& matches,
                          std::vector<cv::Point2f>& alignedLeft,
                          std::vector<cv::Point2f>& alignedRight) {
    alignedLeft.clear();
    alignedRight.clear();

    for (auto const& match : matches) {
        alignedLeft.push_back(leftKeyPoints[match.queryIdx].pt);
        alignedRight.push_back(rightKeyPoints[match.trainIdx].pt);
    }
}