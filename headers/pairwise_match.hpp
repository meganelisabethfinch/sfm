//
// Created by Megan Finch on 13/12/2021.
//

#ifndef SFM_PAIRWISE_MATCH_HPP
#define SFM_PAIRWISE_MATCH_HPP

#include "constants.h"
#include <opencv2/core/types.hpp>

// TODO: match data structure is complex enough to be refactored into its own class
class PairwiseMatch {
private:
    ImageID queryImage;
    ImageID trainImage;

    std::vector<KeyPointIDX> queryIndices; // indices of the matched keypoints in the first image
    std::vector<KeyPointIDX> trainIndices; // indices of the matched keypoints in the second image

    std::vector<float> distances;


public:
    PairwiseMatch(std::vector<cv::DMatch> matches) {
        for (auto& match : matches) {
            queryIndices.push_back(match.queryIdx);
            trainIndices.push_back(match.trainIdx);
            distances.push_back(match.distance);
        }
    }

    std::vector<cv::DMatch> toDMatches() const {
        std::vector<cv::DMatch> dmatches;
        for (size_t i = 0; i < queryIndices.size(); i++) {
            dmatches.push_back(cv::DMatch(queryIndices.at(i), trainIndices.at(i), 0, distances.at(i)));
        }
        return dmatches;
    }


};

#endif //SFM_PAIRWISE_MATCH_HPP
