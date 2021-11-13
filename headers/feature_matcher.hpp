#ifndef FEATURE_MATCHER_H
#define FEATURE_MATCHER_H
#include "image.hpp"
#include <vector>

class FeatureMatcher {
    private:
        double loweRatio;

        bool passesLoweRatioTest(const std::vector<cv::DMatch>& match);

    public:
        FeatureMatcher(double loweRatio) {
            this->loweRatio = loweRatio;
        };
        
        int detect(std::vector<Image> images);
        
        int match(std::vector<Image>& images);

        int getSceneGraph(std::vector<Image>& images);
};
#endif