//
// Created by Megan Finch on 08/12/2021.
//

#ifndef SFM_POINT_CLOUD_HPP
#define SFM_POINT_CLOUD_HPP

using namespace cv;

class PointCloud {
    private:
        std::map<Image*, std::map<size_t, Point3f*>> mapPoints2Dto3D;
        std::map<Point3f*, std::map<Image*, size_t>> mapPoints3Dto2D;

    public:
        std::vector<Point3f> listOfPoints;

        std::vector<Image*> registeredImages;

        Point3f* addPoint(float x, float y, float z) {
            Point3f point = cv::Point3f(x,y,z);
            listOfPoints.push_back(point);
            return &(listOfPoints[listOfPoints.size() - 1]);
        }

        Point3f* addPoint(float x, float y, float z, float w) {
            return addPoint(x / w, y / w, z/ w);
        }

        int updateOriginatingViews(Point3f* point, Image* image, size_t keypoint_idx) {
            mapPoints2Dto3D[image][keypoint_idx] = point;
            mapPoints3Dto2D[point][image] = keypoint_idx;
            return 0;
        }

        void registerImage(Image& image) {
            registeredImages.push_back(&image);
        }

        size_t size() const {
            return listOfPoints.size();
        }

        cv::Point3f* lookupPoint(Image* image, const size_t keypoint_idx) const {
            try {
                return mapPoints2Dto3D.at(image).at(keypoint_idx);
            } catch (std::out_of_range&) {
                return nullptr;
            }
        }

        cv::Point3f* getPointByIndex(size_t i) {
            return &listOfPoints[i];
        }
};

#endif //SFM_POINT_CLOUD_HPP
