#ifndef SFM_TRIANGULATOR_H
#define SFM_TRIANGULATOR_H
#include "image.hpp"
#include "point_cloud.hpp"
#include <vector>

class Triangulator {
    private:
        PointCloud pointCloud;

        std::map<float, ImagePair> sortForBestBaselinePair(std::vector<Image>& images);

    public:
        Triangulator();

        bool computePose(Image& image1, Image& image2, cv::Matx34f& M1, cv::Matx34f& M2, std::vector<cv::DMatch>& prunedMatches);

        void triangulate(Image &image1, Image &image2);

        int computePose(Image& image);

        double calculateReprojectionError(cv::Point3f& point3D, cv::Point2f& point2D, cv::Matx34f& proj);

        int reconstruct(std::vector<Image>& images);

        int pointCloudToPly();

        int exportToCOLMAP(std::vector<Image>& input_images);

        void findBaselineTriangulation(std::vector<Image>& images);
    };


#endif //SFM_TRIANGULATOR_H
