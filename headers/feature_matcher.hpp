#include "image.hpp"

#include <vector>

class FeatureMatcher {
    private:
        double loweRatio;

    public:
        FeatureMatcher(double loweRatio) {
            this->loweRatio = loweRatio;
        };
        
        int match(std::vector<Image> images);
};