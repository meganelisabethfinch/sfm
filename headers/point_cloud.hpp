//
// Created by Megan Finch on 08/12/2021.
//

#ifndef SFM_POINT_CLOUD_HPP
#define SFM_POINT_CLOUD_HPP

using namespace cv;

class PointCloud {
    private:
        std::map<Image*, std::map<size_t, Point3d*>> mapPoints2Dto3D;
        std::map<Point3d*, std::map<Image*, size_t>> mapPoints3Dto2D;
        std::vector<Point3d> listOfPoints;

    public:
        Point3d* addPoint(double x, double y, double z) {
            Point3d point = cv::Point3d(x,y,z);
            listOfPoints.push_back(point);
            return &(listOfPoints[listOfPoints.size() - 1]);
        }

        Point3d* addPoint(double x, double y, double z, double w) {
            return addPoint(x / w, x / y, x / z);
        }

        int updateOriginatingViews(Point3d* point, Image* image, size_t keypoint_idx) {
            mapPoints2Dto3D[image][keypoint_idx] = point;
            mapPoints3Dto2D[point][image] = keypoint_idx;
            return 0;
        }

        size_t size() const {
            return listOfPoints.size();
        }

        cv::Point3d* getPointByIndex(size_t i) {
            return &listOfPoints[i];
        }
};

#endif //SFM_POINT_CLOUD_HPP
