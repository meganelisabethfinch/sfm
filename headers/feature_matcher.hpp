#ifndef FEATURE_MATCHER_H
#define FEATURE_MATCHER_H
#include "image.hpp"
#include <opencv2/core/types.hpp>
#include <vector>

class FeatureMatcher {
    private:
        bool passesLoweRatioTest(const std::vector<cv::DMatch>& match) const;

    public:
        int detect(std::vector<Image>& images);
        
        int match(std::vector<Image>& images);

        int getSceneGraph(std::vector<Image>& images) const;

        static bool enoughMatchesForFundamentalMatrix(const std::vector<cv::DMatch> &goodMatches) ;
};
#endif