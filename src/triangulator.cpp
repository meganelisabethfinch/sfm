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
    std::vector<size_t> points1_idx;
    std::vector<size_t> points2_idx;
    std::vector<Point2f> points1;
    std::vector<Point2f> points2;

    // Probably a better way to store these in the first place
    for (int i = 0; i < image1.keypoints.size(); i++) {
        KeyPoint kp1 = image1.keypoints.at(i);
        try {
            int j = image1.keypoint_matches.at(&image2).at(i);
            KeyPoint kp2 = image2.keypoints.at(j);

            points1_idx.push_back(i);
            points2_idx.push_back(j);

            points1.push_back(kp1.pt);
            points2.push_back(kp2.pt);
        } catch (std::out_of_range) {
            // kp1 in image1 does NOT contain a match in image2
        }
    }

    recoverPose(E, points1, points2, K, R, t);

    // Define projection matrices
    Mat M1; // = [I | 0]
    hconcat(Mat::eye(3,3,CV_32F), Mat::zeros(3, 1, CV_32F), M1);
    Mat M2;
    hconcat(R, t, M2);

    Mat points4D;

    triangulatePoints(M1, M2, points1, points2, points4D);

    std::cout << points4D << std::endl;
    // TODO: use homogenous coords to check if point in front of camera
    for (int i = 0; i < points4D.cols; i++) {

        Point3f* point = pointCloud.addPoint(points4D.at<float>(0, i), points4D.at<float>(1, i), points4D.at<float>(2, i), points4D.at<float>(3,i));

        // Store originating views of points
        pointCloud.updateOriginatingViews(point, &image1, points1_idx[i]);
        pointCloud.updateOriginatingViews(point, &image2, points2_idx[i]);
    }

    registeredImages.push_back(&image1);
    registeredImages.push_back(&image2);

    return 0;
}

int Triangulator::compute_pose(Image &image) {
    std::vector<Point3d> objectPoints;
    std::vector<Point2d> imagePoints;

    for (int i = 0; i < image.keypoints.size(); i++) {
        // check if keypoint already in point cloud

        // if so, add it and objectPoints to imagePoints
    }

    Mat K = Mat::eye(3, 3, CV_32F);
    Mat rvec;
    Mat tvec;

    // Compute pose PnP
    solvePnPRansac(objectPoints, imagePoints, K, NULL, rvec, tvec, false, 100, 8.0, 0.99, noArray(), SOLVEPNP_EPNP);
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
