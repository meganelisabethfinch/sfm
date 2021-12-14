//
// Created by Megan Finch on 13/12/2021.
//

#ifndef SFM_PAIRWISE_MATCHES_HPP
#define SFM_PAIRWISE_MATCHES_HPP

#include "constants.h"
#include <opencv2/core/types.hpp>

class PairwiseMatches {
private:
    ImageID image1;
    ImageID image2;

    std::vector<KeyPointIDX> indices1; // indices of the matched keypoints in the first image
    std::vector<KeyPointIDX> indices2; // indices of the matched keypoints in the second image

    std::vector<float> distances;


public:
    PairwiseMatches(ImageID image1, ImageID image2, std::vector<cv::DMatch> matches) {
        this->image1 = image1;
        this->image2 = image2;
        for (auto& match : matches) {
            indices1.push_back(match.queryIdx);
            indices2.push_back(match.trainIdx);
            distances.push_back(match.distance);
        }
    }

    std::vector<cv::DMatch> toDMatches() const {
        std::vector<cv::DMatch> dmatches;
        for (size_t i = 0; i < indices1.size(); i++) {
            dmatches.push_back(cv::DMatch(indices1.at(i), indices2.at(i), 0, distances.at(i)));
        }
        return dmatches;
    }

};

#endif //SFM_PAIRWISE_MATCHES_HPP
