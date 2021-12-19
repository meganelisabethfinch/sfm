#include "triangulator.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/calib3d.hpp>

#include <point_cloud.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/sfm/triangulation.hpp>

Triangulator::Triangulator() {

}

int Triangulator::reconstruct(std::vector<Image> &images) {
    // Find pair of images with highest number of matches
    ImageID baseline1 = 0;
    ImageID baseline2 = 1;
    int highestMatches = 0;
    for (size_t i = 0; i < images.size() - 1; i++) {
        for (size_t j = i + 1; j < images.size(); j++) {
            if (images[i].countMatchesByImage(j) > highestMatches) {
                baseline1 = i;
                baseline2 = j;
                highestMatches = images[i].countMatchesByImage(j);
            }
        }
    }

    // Compute baseline pose
    compute_pose(images[baseline1], images[baseline2]);
    triangulate(images[baseline1], images[baseline2]);

    /*
    for (int i = 2; i < images.size(); i++) {
        compute_pose(images[i]);
        // then triangulate points
    }
    */

    // Output to file
    pointCloudToPly();

    return 0;
}

int Triangulator::compute_pose(Image &image1, Image &image2) {
    std::cout << "Registering images " << image1.getId() << " and " << image2.getId() << std::endl;
    // For now, use 'ideal' camera matrix
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);

    // Compute essential matrix E
    // E = transpose(K') * E * K
    cv::Mat E = image1.getFundamentalMatrix(image2.getId());

    // Recover pose R and t
    cv::Mat R;
    cv::Mat t;

    std::vector<cv::Point2f> points1 = image1.getMatchedPoints(image2.getId());
    std::vector<cv::Point2f> points2 = image2.getMatchedPoints(image1.getId());

    recoverPose(E, points1, points2, K, R, t);

    // Store matrices
    image1.setPose(cv::Matx34f::eye());

    cv::Matx34f M2 = cv::Matx34f(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
                                 R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                                 R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2));
    image2.setPose(M2);

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

   if (USE_CV_SFM_TRIANGULATION) {
       cv::Mat points3d;

       cv::Mat points1Mat = cv::Mat_<float>(2, 0);
       cv::Mat points2Mat = cv::Mat_<float>(2, 0);

       for (const auto &point: points1) {
           cv::Mat pointAsMatrix = (cv::Mat_<float>(2, 1) << point.x, point.y);
           cv::hconcat(points1Mat, pointAsMatrix, points1Mat);
       }

       for (const auto &point: points2) {
           cv::Mat pointAsMatrix = (cv::Mat_<float>(2, 1) << point.x, point.y);
           cv::hconcat(points2Mat, pointAsMatrix, points2Mat);
       }

       std::vector<cv::Mat> points2d;
       points2d.push_back(points1Mat);
       points2d.push_back(points2Mat);

       std::vector<cv::Mat> projectionMatrices;
       // TODO: resolve compatibility issues between cv:: and cv::sfm
       // it seems cv::sfm likes projection matrices as 3x4 Mat_<double>
       // whereas cv:: likes projection matrices to be Matx34f (float)
       // projectionMatrices.push_back(M1);
       // projectionMatrices.push_back(M2);

       cv::sfm::triangulatePoints(points2d, projectionMatrices, points3d);

       std::vector<double> reprojectionError1;
       std::vector<double> reprojectionError2;

       for (int i = 0; i < points3d.cols; i++) {
           size_t point = pointCloud.addPoint(points3d.at<float>(0, i),
                   points3d.at<float>(1, i),
                   points3d.at<float>(2, i));

           reprojectionError1.push_back(calculateReprojectionError(pointCloud.getPointByIndex(point), points1[i], M1));
           reprojectionError2.push_back(calculateReprojectionError(pointCloud.getPointByIndex(point), points2[i], M2));

           pointCloud.updateOriginatingViews(point, image1.getId(), matches1to2[i].queryIdx);
           pointCloud.updateOriginatingViews(point, image2.getId(), matches1to2[i].trainIdx);
       }
   } else {
       Mat points4D;

       std::vector<double> reprojectionError1;
       std::vector<double> reprojectionError2;

       triangulatePoints(M1, M2, points1, points2, points4D);

       // TODO: use homogenous coords to check if point in front of camera
       for (int i = 0; i < points4D.cols; i++) {

           int point = pointCloud.addPoint(points4D.at<float>(0, i), points4D.at<float>(1, i),
                             points4D.at<float>(2, i), points4D.at<float>(3, i));

           // Store originating views of points
           pointCloud.updateOriginatingViews(point, image1.getId(), matches1to2[i].queryIdx);
           pointCloud.updateOriginatingViews(point, image2.getId(), matches1to2[i].trainIdx);

           reprojectionError1.push_back(calculateReprojectionError(pointCloud.getPointByIndex(point), points1[i], M1));
           reprojectionError2.push_back(calculateReprojectionError(pointCloud.getPointByIndex(point), points2[i], M2));
       }
   }

   pointCloud.registerImage(image1.getId());
   pointCloud.registerImage(image2.getId());
}

int Triangulator::compute_pose(Image &image) {
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
    std::cout << "Original 2D point: " << point2D << std::endl;
    std::cout << "Reprojected point: " << rpEuclidean << std::endl;

    double error = cv::norm(point2D - rpEuclidean);
    return error;
}
