#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <map>
#include <iostream>
#include "pose.h"
#include "constants.h"
#include "pairwise_match.hpp"

class Image {
    private:
        ImageID id;
        std::string name;
        cv::Mat img;

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        std::map<ImageID, std::vector<cv::DMatch>> matches;
        std::map<ImageID, std::map<KeyPointIDX, KeyPointIDX>> matchesByKeyPoint;

        std::map<ImageID, cv::Mat> fundamentalMatrices;
        Pose pose;

public:
        Image(int id, std::string name, cv::Mat img) {
            this->id = id;
            this->name = name;
            this->img = img;
        }

        int getId() const {
            return id;
        }

        std::string getName() const {
            return name;
        }

        cv::Mat& getImage() {
            return img;
        }

        cv::Mat& getDescriptors() {
            return descriptors;
        }

        std::vector<cv::KeyPoint>& getKeyPoints() {
            return keypoints;
        }

        std::vector<cv::DMatch> getMatchesByImage(int imageId) {
            try {
                return matches[imageId];
            } catch (std::out_of_range&) {
                return {};
            }
        }

        /**
         *
         * @param imageId
         * @param kpIdx an index into this->keypoints[kpIdx]
         * @return an index into images[imageId].getKeypoints() of a matching point, or -1
         */
        int getMatchByImageAndKeypoint(int imageId, int kpIdx) const {
            try {
                return matchesByKeyPoint.at(imageId).at(kpIdx);
            } catch (std::out_of_range&) {
                return -1;
            }
        }

        std::vector<cv::KeyPoint> getMatchedKeyPoints(int imageId) {
            try {
                std::vector<cv::DMatch> matchesWith = matches[imageId];
                std::vector<cv::KeyPoint> matchedKeyPoints;
                for (auto &match: matchesWith) {
                    matchedKeyPoints.push_back(keypoints[match.queryIdx]);
                }
                return matchedKeyPoints;
            } catch (std::out_of_range&) {
                // No matches between this image and images[imageId]
                return {};
            }
        }

        std::vector<cv::Point2f> getMatchedPoints(int imageId) {
            try {
                std::vector<cv::DMatch> matchesWith = matches[imageId];
                std::vector<cv::Point2f> matchedPoints;
                for (auto &match: matchesWith) {
                    matchedPoints.push_back(keypoints[match.queryIdx].pt);
                }
                return matchedPoints;
            } catch (std::out_of_range&) {
                return {};
            }
        }

        /**
         * @param imageId
         * @return number of (verified) matches between this image and images[imageId]
         */
        size_t countMatchesByImage(int imageId) {
            try {
                std::vector<cv::DMatch> matchesWith = matches[imageId];
                return matchesWith.size();
            } catch (std::out_of_range&) {
                // No matches with images[imageId]
                return 0;
            }
        }

        cv::Mat& getFundamentalMatrix(ImageID imageId) {
            return fundamentalMatrices.at(imageId);
        }

        Pose& getPose() {
            return pose;
        }

        /**
         *
         * @param imageId
         * @param newMatches
         * @param query true when the queryIdx in DMatches indexes keypoints in *this* image,
         * and trainIdx indexes keypoints in images[imageId]; false when the opposite is true
         */
        void setMatches(int imageId, std::vector<cv::DMatch>& newMatches, bool query = true) {
            if (query) {
                for (const auto &match: newMatches) {
                    matchesByKeyPoint[imageId][match.queryIdx] = match.trainIdx;
                }
                matches[imageId] = newMatches;
            } else {
                std::vector<cv::DMatch> flippedMatches;
                for (const auto &match : newMatches) {
                    flippedMatches.push_back(cv::DMatch(match.trainIdx, match.queryIdx, 0, match.distance));
                    matchesByKeyPoint[imageId][match.trainIdx] = match.queryIdx;
                }
                matches[imageId] = flippedMatches;
            }
        }

        void setFundamentalMatrix(int imageId, cv::Mat& F) {
            fundamentalMatrices[imageId] = F;
        }

        void setKeyPointsAndDescriptors(std::vector<cv::KeyPoint> keypoints, cv::Mat descriptors) {
            this->keypoints = std::move(keypoints);
            this->descriptors = std::move(descriptors);
        }

        void setPose(cv::Mat R, cv::Mat t) {
            pose.R = R;
            pose.t = t;
        }
};
#endif