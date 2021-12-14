//
// Created by Megan Finch on 08/12/2021.
//

#ifndef SFM_POINT_CLOUD_HPP
#define SFM_POINT_CLOUD_HPP

using namespace cv;

typedef int PointIDX;

class PointCloud {
    private:
        std::map<ImageID, std::map<KeyPointIDX, Point3f*>> mapPoints2Dto3D;
        std::map<Point3f*, std::map<ImageID, KeyPointIDX>> mapPoints3Dto2D;

        std::vector<Point3f> listOfPoints;
        std::vector<ImageID> registeredImages;

        std::vector<float> errors; // list of mean reprojection errors

public:
        Point3f* addPoint(float x, float y, float z) {
            Point3f point = cv::Point3f(x,y,z);
            listOfPoints.push_back(point);
            return &(listOfPoints[listOfPoints.size() - 1]);
        }

        Point3f* addPoint(float x, float y, float z, float w) {
            return addPoint(x / w, y / w, z/ w);
        }

        int updateOriginatingViews(Point3f* point, ImageID image, KeyPointIDX kpIdx) {
            mapPoints2Dto3D[image][kpIdx] = point;
            mapPoints3Dto2D[point][image] = kpIdx;
            return 0;
        }

        void registerImage(int imageId) {
            registeredImages.push_back(imageId);
        }

        size_t size() const {
            return listOfPoints.size();
        }

        cv::Point3f* lookupPoint(ImageID image, const KeyPointIDX kpIdx) const {
            try {
                return mapPoints2Dto3D.at(image).at(kpIdx);
            } catch (std::out_of_range&) {
                return nullptr;
            }
        }

        cv::Point3f* getPointByIndex(size_t i) {
            return &listOfPoints[i];
        }

        std::vector<ImageID> getRegisteredImageIDs() {
            return registeredImages;
        }
};

#endif //SFM_POINT_CLOUD_HPP
