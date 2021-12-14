#include "feature_matcher.hpp"
#include "constants.h"

#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>
#include <iostream>

using namespace cv;

bool FeatureMatcher::passesLoweRatioTest(const std::vector<DMatch>& match) const {
    return match.size() == 2 && match[0].distance < match[1].distance * LOWE_RATIO;
}

int FeatureMatcher::detect(std::vector<Image>& images) {
    Ptr<SIFT> detector = SIFT::create();

    for (size_t i = 0; i < images.size(); i++) {
        std::vector<KeyPoint> keypoints;
        Mat descriptors;

        detector->detectAndCompute(images[i].getImage(), noArray(), keypoints, descriptors);

        images[i].setKeyPointsAndDescriptors(keypoints, descriptors);
    }

    return 0;
}

int FeatureMatcher::match(std::vector<Image>& images) {
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    for (int i = 0; i < images.size() - 1; i++) {
        for (int j = i+1; j < images.size(); j++) {
            std::vector<std::vector<DMatch>> matches;
            std::vector<Point2f> source, destination;

            matcher->knnMatch(images[i].getDescriptors(),
                               images[j].getDescriptors(), matches, 2);
            
            std::vector<DMatch> loweRatioMatches;

            for (auto& match : matches) {
                if (passesLoweRatioTest(match)) {
                    loweRatioMatches.push_back(match[0]);

                    // Track coordinates of keypoints involved in good matches
                    source.push_back(images[i].getKeyPoints()[match[0].queryIdx].pt);
                    destination.push_back(images[j].getKeyPoints()[match[0].trainIdx].pt);
                }
            }

            std::vector<DMatch> verifiedMatches;

            // Geometric verification by fundamental matrix
            if (enoughMatchesToFindMatrix(loweRatioMatches)) {
                std::vector<uchar> mask;
                Mat F = findFundamentalMat(source, destination, FM_RANSAC, 3.0, 0.99, mask);

                // Store fundamental matrix
                // TODO: should one be F.inverse?
                images[i].setFundamentalMatrix(j, F);
                images[j].setFundamentalMatrix(i, F);

                for (int k = 0; k < mask.size(); k++) {
                    if (mask[i]) {
                        // Classify this as a good match
                        verifiedMatches.push_back(loweRatioMatches[k]);

                        // images[i].keypoint_matches[&images[j]][goodMatches[matchIdx].queryIdx] = goodMatches[matchIdx].trainIdx;
                        // images[j].keypoint_matches[&images[i]][goodMatches[matchIdx].trainIdx] = goodMatches[matchIdx].queryIdx;
                    }
                }
            }

            // Store matches
            images[i].setMatches(j, verifiedMatches);
            images[j].setMatches(i, verifiedMatches, false);

            // Output example matches for debug
            if (DEBUG_MODE) {
                if (i == 0 & j == 1) {
                    Mat outImg;
                    drawMatches(images[i].getImage(), images[i].getKeyPoints(), images[j].getImage(),
                                images[j].getKeyPoints(), verifiedMatches, outImg);
                    imshow("Matches", outImg);
                    waitKey(0);
                }
            }
        }
    }

    return 0;
}

bool
FeatureMatcher::enoughMatchesToFindMatrix(const std::vector<DMatch> &goodMatches) const { return goodMatches.size() >= 16; }

int FeatureMatcher::getSceneGraph(std::vector<Image>& images) const {
    std::cout << "Constructing scene graph..." << std::endl;

    std::ofstream graph ("./data/out/scene_graph.dot");

    graph << "graph sceneGraph {" << std::endl;

    for (size_t i = 0; i < images.size() - 1; i++) {
        for (size_t j = i + 1; j < images.size(); j++) {
            size_t matchCount = images[i].countMatchesByImage(j);

            if (matchCount > MATCH_COUNT_THRESHOLD) {
                graph << images[i].getName() << " -- " << images[j].getName() << std::endl;
            }
        }
    }

    graph << "}" << std::endl;
    graph.close();

    return 0;
}