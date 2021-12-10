#ifndef SFM_TRIANGULATOR_H
#define SFM_TRIANGULATOR_H
#include "image.hpp"
#include "point_cloud.hpp"
#include <vector>

class Triangulator {
    private:
        PointCloud pointCloud;
        std::vector<Image*> registeredImages;

    public:
        Triangulator();

        int compute_pose(Image& image1, Image& image2);

        int compute_pose(Image& image);

        int reconstruct(std::vector<Image>& images);

        int pointCloudToPly();
};


#endif //SFM_TRIANGULATOR_H
