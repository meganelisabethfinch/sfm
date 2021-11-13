#include "feature_matcher.hpp"

#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>
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
            if (goodMatches.size() >= 7) {
                std::vector<uchar> mask;
                Mat F = findFundamentalMat(source, destination, FM_RANSAC, 3.0, 0.99, mask);
            
                for (int matchIdx = 0; matchIdx < mask.size(); matchIdx++) {
                    if (mask[i]) {
                        // Classify this as a good match
                        // images[i].keypoint_matches[goodMatches[matchIdx].queryIdx][j] = goodMatches[matchIdx].trainIdx;
                        // images[i].keypoint_matches[goodMatches[matchIdx].trainIdx][i] = goodMatches[matchIdx].queryIdx;

                        images[i].keypoint_matches[j][goodMatches[matchIdx].queryIdx] = goodMatches[matchIdx].trainIdx;
                        images[j].keypoint_matches[i][goodMatches[matchIdx].trainIdx] = goodMatches[matchIdx].queryIdx;
                    }
                }
            }

            // Output example matches for debug
            /*
            if (i == 0 & j == 8) {
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

int FeatureMatcher::getSceneGraph(std::vector<Image> images) {
    std::cout << "Constructing scene graph..." << std::endl;

    std::ofstream graph ("./data/out/test.txt");

    graph << "graph sceneGraph {" << std::endl;

    for (int i = 0; i < images.size() - 1; i++) {
        for (int j = i + 1; j < images.size(); j++) {
            int count = 0;
            
            std::cout << "Get matches map for images " << i << " and " << j << std::endl;
            std::map<int,int> matches_ij = images[i].keypoint_matches.at(j);

            // count matches
            for (int kp = 0; kp < images[i].keypoints.size(); kp++) {
                try {
                    int kp2 = matches_ij.at(kp);
                    count++;
                } catch(std::out_of_range) {
                    // kp does not contain a match in image j
                }
            }

            std::cout << "image " << i << " matches with image " << j << ", " << count << " times" << std::endl;

            // if image i has enough matches with image j
            // TODO: make this (25) a parameter
            if (count > 25) {
                graph << images[i].name << " -- " << images[j].name << std::endl;
            }
            
        }
    }

    graph << "}" << std::endl;

    graph.close();

    return 0;
}