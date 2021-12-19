//
// Created by Megan Finch on 08/12/2021.
//

#ifndef SFM_POINT_CLOUD_HPP
#define SFM_POINT_CLOUD_HPP

using namespace cv;

class PointCloud {
    private:
        // Map ImageID -> KeyPointID -> 3D Point ID
        std::map<ImageID, std::map<PointIDX, PointIDX>> mapPoints2Dto3D;
        // Map 3D Point ID -> ImageID -> KeyPointID
        std::map<PointIDX, std::map<ImageID, PointIDX>> mapPoints3Dto2D;

        std::vector<Point3f> listOfPoints;
        std::vector<ImageID> registeredImages;

        std::vector<float> errors; // list of mean reprojection errors

public:
        PointIDX addPoint(const cv::Point3f& point) {
            listOfPoints.push_back(point);
            return listOfPoints.size() - 1;
        }

        int updateOriginatingViews(PointIDX point, ImageID image, PointIDX kpIdx) {
            mapPoints2Dto3D[image][kpIdx] = point;
            mapPoints3Dto2D[point][image] = kpIdx;
            return 0;
        }

        void registerImage(int imageId) {
            registeredImages.push_back(imageId);
        }

        [[nodiscard]] size_t size() const {
            return listOfPoints.size();
        }

        /**
         * Returns the identifier (index) of the 3D point associated with a given keypoint,
         * or -1 if no such point exists.
         * @param image
         * @param kpIdx
         * @return
         */
        [[nodiscard]] int lookupPoint(ImageID image, const PointIDX kpIdx) const {
            try {
                return mapPoints2Dto3D.at(image).at(kpIdx);
            } catch (std::out_of_range&) {
                return -1;
            }
        }

        cv::Point3f& getPointByIndex(size_t i) {
            return listOfPoints[i];
        }

        std::vector<ImageID> getRegisteredImageIDs() {
            return registeredImages;
        }
};

#endif //SFM_POINT_CLOUD_HPP
