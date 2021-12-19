//
// Created by Megan Finch on 19/12/2021.
//

#ifndef SFM_SFM_UTILITIES_HPP
#define SFM_SFM_UTILITIES_HPP

#include "image.hpp"

/**
 *
 * @param image1
 * @param image2
 * @return the number of homography inliers between the two images
 */
int findHomographyInliers(Image& image1, Image& image2);

void getPointsFromMatches(std::vector<cv::KeyPoint>& leftKeyPoints,
                          std::vector<cv::KeyPoint>& rightKeyPoints,
                          std::vector<cv::DMatch>& matches,
                          std::vector<cv::Point2f>& alignedLeft,
                          std::vector<cv::Point2f>& alignedRight);

#endif //SFM_SFM_UTILITIES_HPP
