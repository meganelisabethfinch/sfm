#include "feature_matcher.hpp"

#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

using namespace cv;

int FeatureMatcher::detect(std::vector<Image> images) {
    return 0;
}

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
                    goodMatches.push_back(match[0]);

                    // Track coordinates of keypoints involved in good matches
                    source.push_back(images[i].keypoints[match[0].queryIdx].pt);
                    destination.push_back(images[j].keypoints[match[0].trainIdx].pt);
                }
            }
            
            // Geometric verification by fundamental matrix
            if (goodMatches.size() > 0) {
                std::vector<uchar> mask;
                Mat F = findFundamentalMat(source, destination, FM_RANSAC, 3.0, 0.99, mask);

                // std::cout << goodMatches.size() << std::endl;
                // std::cout << mask.size() << std::endl;
                
                if (i == 0 && j == 1) {
                    std::cout << goodMatches.size() << std::endl;
                    std::cout << mask.size() << std::endl;
                    for (int k = 0; k < goodMatches.size(); k++) {
                        std::cout << mask[k] << std::endl;
                    }
                }
            
                for (int matchIdx = 0; matchIdx < goodMatches.size(); matchIdx++) {
                    images[i].keypoint_matches[goodMatches[matchIdx].queryIdx][j] = goodMatches[matchIdx].trainIdx;
                    images[i].keypoint_matches[goodMatches[matchIdx].trainIdx][i] = goodMatches[matchIdx].queryIdx;
                }
            }

            // Output example matches for debug
            /*
            if (i == 0 & j == 1) {
                Mat outImg;
                drawMatches(images[i].img, images[i].keypoints, images[j].img, images[j].keypoints, goodMatches, outImg);
                imshow("Matches", outImg);
                waitKey(0);
            }
            */
        }
    }

    return 0;
}