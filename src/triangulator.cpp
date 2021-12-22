#include "triangulator.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/calib3d.hpp>

#include <point_cloud.hpp>
#include <iostream>
#include <fstream>
#include <sfm_utilities.hpp>
#include <data_structures.h>

Triangulator::Triangulator() {

}

int Triangulator::reconstruct(std::vector<Image> &images) {
    findBaselineTriangulation(images);

    /*
    for (int i = 2; i < images.size(); i++) {
        computePose(images[i]);
        // then triangulate points
    }
    */

    // Output to file
    pointCloudToPly();

    return 0;
}

void Triangulator::triangulate(Image &image1, Image &image2) {
    // Define projection matrices
    cv::Matx34f M1 = image1.getPose();
    cv::Matx34f M2 = image2.getPose();

    std::vector<cv::Point2f> points1 = image1.getMatchedPoints(image2.getId());
    std::vector<cv::Point2f> points2 = image2.getMatchedPoints(image1.getId());

    // The query/train indices of these points are also needed, but only to update originating views
    std::vector<DMatch> matches1to2 = image1.getMatchesByImage(image2.getId());

    Mat points3dHomogeneous;

    std::vector<double> reprojectionError1;
    std::vector<double> reprojectionError2;

    cv::triangulatePoints(M1, M2, points1, points2, points3dHomogeneous);

    Mat points3d;
    cv::convertPointsFromHomogeneous(points3dHomogeneous.t(), points3d);

    // TODO: use homogenous coords to check if point in front of camera
    for (size_t i = 0; i < points3d.rows; i++) {
        cv::Point3f point = cv::Point3f(points3d.at<float>(i,0),
                                        points3d.at<float>(i,1),
                                        points3d.at<float>(i,2));
        double error1 = calculateReprojectionError(point, points1[i], M1);
        double error2 = calculateReprojectionError(point, points2[i], M2);

        // Ignore points with high reprojection error
        if (error1 > MAX_REPROJECTION_ERROR or error2 > MAX_REPROJECTION_ERROR) {
            continue;
        } else {
            int pointIDX = pointCloud.addPoint(point);

            // Store originating views of points
            pointCloud.updateOriginatingViews(pointIDX, image1.getId(), matches1to2[i].queryIdx);
            pointCloud.updateOriginatingViews(pointIDX, image2.getId(), matches1to2[i].trainIdx);
        }
    }

   pointCloud.registerImage(image1.getId());
   pointCloud.registerImage(image2.getId());
}

int Triangulator::computePose(Image &image) {
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;

    /*
    for (auto const& oldView : pointCloud.registeredImages) {
        try {
            std::map<int, int> matches = image.keypoint_matches[oldView];

            for (auto& [kp1_idx,kp2_idx] : matches) {
                Point3f* point3F = pointCloud.lookupPoint(oldView, kp2_idx);

                // If point exists AND is not already in objectPoints
                if (point3F != nullptr
                    && std::find(objectPointsIdx.begin(), objectPointsIdx.end(), point3F) == objectPointsIdx.end()) {
                    objectPoints.push_back(*point3F);
                    imagePoints.push_back(image.getKeyPoints()[kp1_idx].pt);
                    objectPointsIdx.push_back(point3F);
                }
            }
        } catch (std::out_of_range&) {
            // No matches between new image and oldView
        }
    }
     */

    /*
    // Collect all descriptors of registered images
    Mat oldDescriptors;
    for (auto const& oldView : pointCloud.registeredImages) {
        hconcat(oldDescriptors, oldView->descriptors, oldDescriptors);
    }

    // matches old descriptors against descriptors in the new view
    */

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F); // or CV_64F?
    cv::Mat rvec;
    cv::Mat tvec;

    // Compute pose PnP
    cv::solvePnPRansac(objectPoints,imagePoints,K,
                   NULL,rvec,tvec,
                   false,100,8.0,
                   0.99, noArray(),SOLVEPNP_EPNP);

    // cv::Rodrigues(rvec, image.getPose().R);
    // image.getPose().t = tvec;

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
        file << pointCloud.getPointByIndex(pt).x << " ";
        file << pointCloud.getPointByIndex(pt).y << " ";
        file << pointCloud.getPointByIndex(pt).z << std::endl;
    }

    file.close();

    return 0;
}

int Triangulator::exportToCOLMAP(std::vector<Image>& input_images) {
    std::cout << "Exporting to COLMAP..." << std::endl;

    // For now, one camera per image
    std::ofstream cameras ("./data/out/colmap_export/cameras.txt");
    for (auto const& cameraID : pointCloud.getRegisteredImageIDs()) {
        // Camera list with one line of data per camera:
        // CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
        cameras << cameraID << " "
                << "SIMPLE_PINHOLE" << " "
                << std::endl;
    }
    cameras.close();

    std::ofstream images ("./data/out/colmap_export/images.txt");
    for (auto const& imageID : pointCloud.getRegisteredImageIDs()) {
        // Convert pose.rotation to quaternion, using Hamilton convention
        // Eigen library does this

        // IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        images << imageID << " "
        //     << quaternion << " "
               << (input_images[imageID].getPose()(0,3)) << " " // TX
               << (input_images[imageID].getPose()(1,3)) << " " // TY
               << (input_images[imageID].getPose()(2,3)) << " " // TZ
               << imageID << " " // CAMERA_ID
               << input_images[imageID].getName() // NAME
               << std::endl;

        // POINTS2D as (X, Y, POINT3D_ID)
        /*
        std::vector<KeyPoint> keypoints = input_images[imageID].getKeyPoints();
        for (int i = 0; i < keypoints.size(); i++) {
            images << keypoints[i].pt.x << " "
                   << keypoints[i].pt.y << " "
                   << pointCloud.lookupPoint(imageID, i) << " ";
        }
         */
        images << std::endl;
    }
    images.close();

    /*
    std::ofstream points3D ("./data/out/colmap_export/points3D.txt");
    for (auto const& point : pointCloud.getListOfPoints) {
        // POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)

    }
    points3D.close();
     */

    return 0;
}

double Triangulator::calculateReprojectionError(Point3f &point3D, Point2f &point2D, cv::Matx34f& proj) {

    cv::Mat point3Dvec = (cv::Mat_<float>(4, 1) << point3D.x, point3D.y, point3D.z, 1);
    cv::Mat rp = proj * point3Dvec; // reprojected point (homogenous)
    cv::Point2f rpEuclidean = cv::Point2f(rp.at<float>(0,0) / rp.at<float>(0,2),
            rp.at<float>(0,1) / rp.at<float>(0,2));

    std::cout << "----" << std::endl;
    std::cout << "3D point: " << point3D << std::endl;
    std::cout << "Original 2D point: " << point2D << std::endl;
    std::cout << "Reprojected point: " << rpEuclidean << std::endl;

    double error = cv::norm(point2D - rpEuclidean);
    return error;
}

void Triangulator::findBaselineTriangulation(std::vector<Image> &images) {
    std::cout << "Finding baseline triangulation..." << std::endl;

    std::map<float, ImagePair> pairsByHomographyInlierRatio = sortForBestBaselinePair(images);

    cv::Matx34f M1 = cv::Matx34f::eye();
    cv::Matx34f M2 = cv::Matx34f::eye();

    for (auto& pair : pairsByHomographyInlierRatio) {
        std::cout << "Trying images " << pair.second.left << " and " << pair.second.right << " with ratio " << pair.first << std::endl;

        ImageID i = pair.second.left;
        ImageID j = pair.second.right;

        std::vector<cv::DMatch> matches = images[i].getMatchesByImage(j);
        std::vector<cv::DMatch> prunedMatches;
        bool success = computePose(images[i], images[j], M1, M2, prunedMatches);

        if (not success) {
            // Could not confidently compute pose for these images
            continue;
        }

        float ratioPoseInliers = ((float) prunedMatches.size()) / ((float) matches.size());

        if (ratioPoseInliers < MIN_RATIO_POSE_INLIERS) {
            // Not enough pose inliers. Skip.
            continue;
        }

        images[i].setMatches(j, prunedMatches);
        images[j].setMatches(i, prunedMatches, false);

        std::cout << M1 << std::endl;
        images[i].setPose(M1);
        images[j].setPose(M2);

        triangulate(images[i], images[j]);
        break;
    }
}

std::map<float, ImagePair> Triangulator::sortForBestBaselinePair(std::vector<Image> &images) {
    std::map<float, ImagePair> pairsByHomographyInlierRatio;

    for (size_t i = 0; i < images.size() - 1; i++) {
        for (size_t j = i + 1; j < images.size(); j++) {
            int totalMatches = images[i].countMatchesWithImage(j);

            if (totalMatches < MIN_POINTS_FOR_HOMOGRAPHY) {
                // Not enough points to get good homography
                continue;
            }
            int numInliers = SFMUtilities::findHomographyInliers(images[i], images[j]);
            float ratioInliers = ((float) numInliers) / ((float) totalMatches);

            pairsByHomographyInlierRatio[ratioInliers] = { i, j };
        }
    }

    return pairsByHomographyInlierRatio;
}

bool Triangulator::computePose(Image &image1, Image &image2, Matx34f &M1, Matx34f &M2,
                               std::vector<cv::DMatch> &prunedMatches) {

    std::vector<cv::Point2f> points1 = image1.getMatchedPoints(image2.getId());
    std::vector<cv::Point2f> points2 = image2.getMatchedPoints(image1.getId());

    cv::Mat F = image1.getFundamentalMatrix(image2.getId());
    cv::Mat K1;
    image1.getCameraMatrix().convertTo(K1, CV_64F);
    cv::Mat K2;
    image2.getCameraMatrix().convertTo(K2, CV_64F);
    cv::Mat E = K2.t() * F * K1;

    // Assumes image1 and image2 have the same camera matrix
    // cv::Mat E = cv::findEssentialMat(points1, points2, K1, cv::RANSAC, 0.99, 1.0, 1000, noArray());

    // Recover pose R and t
    cv::Mat R;
    cv::Mat t;
    Mat mask;

    // cv::recoverPose(points1, points2, K1, distCoeffs1, K2, distCoeffs2, E, R, t, RANSAC, 0.99, 1.0, mask);
    recoverPose(E, points1, points2, K1, R, t, mask);

    M1 = cv::Matx34f::eye();
    M2 = cv::Matx34f(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0),
                     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2));

    std::vector<cv::DMatch> matches = image1.getMatchesByImage(image2.getId());
    prunedMatches.clear();
    for (size_t i = 0; i < mask.rows; i++) {
        if (mask.at<uchar>(i)) {
            prunedMatches.push_back(matches[i]);
        }
    }
    return true;
}
