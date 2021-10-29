#include "feature_matcher.hpp"

#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>

using namespace cv;

int FeatureMatcher::match(std::vector<Image> images) {
    
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    for (int i = 0; i < images.size() - 1; i++) {
        for (int j = i+1; j < images.size(); j++) {
            std::vector<std::vector<DMatch>> matches;
            std::vector<Point2f> source, destination;
            matcher->knnMatch(images[i].descriptors, images[j].descriptors, matches, 2);
            
            std::vector<DMatch> goodMatches;

            for (auto& match : matches) {
                // Apply Lowe's Ratio Test
                if (match.size() == 2 && match[0].distance < match[1].distance * loweRatio) {
                    // Keep m[0] as a good match
                    goodMatches.push_back(match[0]);

                    // Track coordinates of keypoints involved in good matches
                    source.push_back(images[i].keypoints[match[0].queryIdx].pt);
                    destination.push_back(images[j].keypoints[match[0].trainIdx].pt);
                }
            }

            std::cout << goodMatches.size() << std::endl;
            
            Mat F = findFundamentalMat(source, destination, FM_RANSAC, 3.0, 0.99, noArray());

            std::cout << "Found fundamental matrix" << std::endl;
        }
    }

    return 0;
}