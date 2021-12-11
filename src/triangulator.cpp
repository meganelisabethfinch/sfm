#include "triangulator.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/calib3d.hpp>
#include <point_cloud.hpp>
#include <iostream>
#include <fstream>

using namespace cv;

Triangulator::Triangulator() {

}

int Triangulator::reconstruct(std::vector<Image> &images) {

    // Compute baseline pose
    compute_pose(images[0], images[1]);
    triangulate(images[0], images[1]);

    for (int i = 2; i < images.size(); i++) {
        compute_pose(images[i]);
        // then triangulate points
    }

    // Output to file
    pointCloudToPly();

    return 0;
}

int Triangulator::compute_pose(Image &image1, Image &image2) {
    // For now, use 'ideal' camera matrix
    Mat K = Mat::eye(3, 3, CV_32F);

    // Compute essential matrix E
    // E = transpose(K') * E * K
    Mat E = image1.fundamentalMatrices.at(&image2);

    // Recover pose R and t
    Mat R;
    Mat t;

    std::vector<Point2f> points1;
    std::vector<Point2f> points2;

    std::map<int, int> matches = image1.keypoint_matches[&image2];

    for (auto const& [key, val] : matches) {
        points1.push_back(image1.keypoints[key].pt);
        points2.push_back(image2.keypoints[val].pt);
    }

    recoverPose(E, points1, points2, K, R, t);

    // Store matrices
    image1.pose.R = Mat::eye(3,3,CV_32F);
    image1.pose.t = Mat::zeros(3,1,CV_32F);

    image2.pose.R = R;
    image2.pose.t = t;

    return 0;
}

std::vector<Point2f> getPointsFromIndices(std::vector<KeyPoint> kp1, std::vector<KeyPoint> kp2, std::vector<DMatch> matches) {

}

int Triangulator::triangulate(Image &image1, Image &image2) {
    // Define projection matrices
    Mat M1; // = [I | 0]
    hconcat(image1.pose.R, image1.pose.t, M1);
    Mat M2;
    hconcat(image2.pose.R, image2.pose.t, M2);

    std::vector<size_t> points1_idx;
    std::vector<size_t> points2_idx;
    std::vector<Point2f> points1;
    std::vector<Point2f> points2;
    std::map<int, int> matches = image1.keypoint_matches[&image2];

    for (auto const& [key, val] : matches) {
        points1_idx.push_back(key);
        points2_idx.push_back(val);
        points1.push_back(image1.keypoints[key].pt);
        points2.push_back(image2.keypoints[val].pt);
    }

    Mat points4D;

    triangulatePoints(M1, M2, points1, points2, points4D);

    // TODO: use homogenous coords to check if point in front of camera
    for (int i = 0; i < points4D.cols; i++) {

        Point3f* point = pointCloud.addPoint(points4D.at<float>(0, i), points4D.at<float>(1, i), points4D.at<float>(2, i), points4D.at<float>(3,i));

        // Store originating views of points
        pointCloud.updateOriginatingViews(point, &image1, points1_idx[i]);
        pointCloud.updateOriginatingViews(point, &image2, points2_idx[i]);
    }

    pointCloud.registerImage(image1);
    pointCloud.registerImage(image2);
}

int Triangulator::compute_pose(Image &image) {
    std::vector<Point3f> objectPoints;
    std::vector<Point2f> imagePoints;
    std::vector<Point3f*> objectPointsIdx;

    for (auto const& oldView : pointCloud.registeredImages) {
        try {
            std::map<int, int> matches = image.keypoint_matches[oldView];

            for (auto& [kp1_idx,kp2_idx] : matches) {
                Point3f* point3F = pointCloud.lookupPoint(oldView, kp2_idx);

                // If point exists AND is not already in objectPoints
                if (point3F != nullptr
                    && std::find(objectPointsIdx.begin(), objectPointsIdx.end(), point3F) == objectPointsIdx.end()) {
                    objectPoints.push_back(*point3F);
                    imagePoints.push_back(image.keypoints[kp1_idx].pt);
                    objectPointsIdx.push_back(point3F);
                }
            }
        } catch (std::out_of_range&) {
            // No matches between new image and oldView
        }
    }

    /*
    // Collect all descriptors of registered images
    Mat oldDescriptors;
    for (auto const& oldView : pointCloud.registeredImages) {
        hconcat(oldDescriptors, oldView->descriptors, oldDescriptors);
    }

    // Match old descriptors against descriptors in the new view
    */

    Mat K = Mat::eye(3, 3, CV_32F);
    Mat rvec;
    Mat tvec;

    // Compute pose PnP
    solvePnPRansac(objectPoints,imagePoints,K,
                   NULL,rvec,tvec,
                   false,100,8.0,
                   0.99, noArray(),SOLVEPNP_EPNP);

    Rodrigues(rvec, image.pose.R);
    image.pose.t = tvec;

    return 0;
}

int Triangulator::pointCloudToPly() {
    std::cout << "Converting point cloud to file..." << std::endl;

    std::ofstream file ("./data/out/point_cloud.ply");
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << pointCloud.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    // Other properties like colour
    file << "end_header" << std::endl;

    for (size_t pt = 0; pt < pointCloud.size(); pt++) {
        file << pointCloud.getPointByIndex(pt)->x << " ";
        file << pointCloud.getPointByIndex(pt)->y << " ";
        file << pointCloud.getPointByIndex(pt)->z << std::endl;
    }

    file.close();

    return 0;
}
