#ifndef SFM_TRIANGULATOR_H
#define SFM_TRIANGULATOR_H
#include "image.hpp"
#include "point_cloud.hpp"
#include <vector>

class Triangulator {
    private:
        PointCloud pointCloud;

    public:
        Triangulator();

        int compute_pose(Image& image1, Image& image2);

        void triangulate(Image &image1, Image &image2);

        int compute_pose(Image& image);

        double calculateReprojectionError(cv::Point3f& point3D, cv::Point2f& point2D, cv::Mat& projectionMatrix);

        int reconstruct(std::vector<Image>& images);

        int pointCloudToPly();

        int exportToCOLMAP();
};


#endif //SFM_TRIANGULATOR_H
