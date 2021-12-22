#include "image.hpp"
#include "feature_matcher.hpp"

#include <set>
#include <string>
#include <unistd.h>
#include <iostream>
#include <filesystem>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
//#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
// #include <opencv2/sfm/triangulation.hpp>

/***********************************************************************************************************************
 *  SIMPLE SFM IMPLEMENTATION
 *  DOES NOT DO BUNDLE ADJUSTMENT, BUT WORKS AS A PROOF OF CONCEPT FOR A FEW CAMERAS WITH SIMILAR POSES
 *  BASED ON REFERENCE IMPLEMENTATION HERE: https://github.com/royshil/SfM-Toy-Library/
 *  (MIT LICENSED)
*/

///Rotational element in a 3x4 matrix
const cv::Rect ROT(0, 0, 3, 3);

///Translational element in a 3x4 matrix
const cv::Rect TRA(3, 0, 1, 3);

///Minimal ratio of inliers-to-total number of points for computing camera pose
const float POSE_INLIERS_MINIMAL_RATIO = 0.5;

struct Intrinsics {
    cv::Mat K;
    cv::Mat Kinv;
    cv::Mat distortion;
};

std::ostream& operator<< (std::ostream& stream, const ImagePair& pair);

typedef std::vector<cv::KeyPoint> Keypoints;
typedef std::vector<cv::Point2f>  Points2f;
typedef std::vector<cv::Point3f>  Points3f;

struct Image2D3DMatch {
    Points2f points2D;
    Points3f points3D;
};

struct Features {
    Keypoints keyPoints;
    Points2f  points;
    cv::Mat   descriptors;
};

struct Point3DInMap {
    // 3D point.
    cv::Point3f p;

    // A mapping from image index to 2D point index in that image's list of features.
    std::map<int, int> originatingViews;
};

struct Point3DInMapRGB {
    Point3DInMap p;
    cv::Scalar   rgb;
};

typedef std::vector<cv::DMatch>      Matching2;


typedef std::vector<Point3DInMap>    PointCloud;
typedef std::vector<Point3DInMapRGB> PointCloudRGB;

typedef cv::Matx34f Pose;
typedef std::vector<std::vector<Matching2> > MatchMatrix;
typedef std::map<int, Image2D3DMatch> Images2D3DMatches;


namespace fs = std::filesystem;

std::vector<cv::Mat>      mImages;
std::vector<Features>     mImageFeatures;
std::vector<cv::Matx34f>  mCameraPoses;
std::set<int>             mDoneViews;
std::set<int>             mGoodViews;
MatchMatrix               mFeatureMatchMatrix;
Intrinsics                mIntrinsics;
PointCloud                mReconstructionCloud;

double meanReprojectionError = 0.0;
double minReprojectionError = std::numeric_limits<double>::infinity();
double maxReprojectionError = 0;
int numReprojections = 0;



void KeyPointsToPoints(const Keypoints& kps, Points2f& ps) {
    ps.clear();
    for (const auto& kp : kps) {
        ps.push_back(kp.pt);
    }
}

void GetAlignedPointsFromMatch(const Features& leftFeatures,
                               const Features& rightFeatures,
                               const Matching2& matches,
                               Features& alignedLeft,
                               Features& alignedRight,
                               std::vector<int>& leftBackReference,
                               std::vector<int>& rightBackReference)
{
    alignedLeft .keyPoints.clear();
    alignedRight.keyPoints.clear();
    alignedLeft .descriptors = cv::Mat();
    alignedRight.descriptors = cv::Mat();

    for (unsigned int i=0; i<matches.size(); i++) {
        alignedLeft .keyPoints  .push_back(leftFeatures.keyPoints       [matches[i].queryIdx]);
        alignedLeft .descriptors.push_back(leftFeatures.descriptors.row (matches[i].queryIdx));
        alignedRight.keyPoints  .push_back(rightFeatures.keyPoints      [matches[i].trainIdx]);
        alignedRight.descriptors.push_back(rightFeatures.descriptors.row(matches[i].trainIdx));
        leftBackReference .push_back(matches[i].queryIdx);
        rightBackReference.push_back(matches[i].trainIdx);
    }

    KeyPointsToPoints(alignedLeft.keyPoints,  alignedLeft.points);
    KeyPointsToPoints(alignedRight.keyPoints, alignedRight.points);
}


void GetAlignedPointsFromMatch(const Features& leftFeatures,
                               const Features& rightFeatures,
                               const Matching2& matches,
                               Features& alignedLeft,
                               Features& alignedRight)
{
    std::vector<int> leftBackReference, rightBackReference;
    GetAlignedPointsFromMatch(
            leftFeatures,
            rightFeatures,
            matches,
            alignedLeft,
            alignedRight,
            leftBackReference,
            rightBackReference
            );

}


int findHomographyInliers(
        const Features& left,
        const Features& right,
        const Matching2& matches) {

    Features alignedLeft;
    Features alignedRight;
    GetAlignedPointsFromMatch(left, right, matches, alignedLeft, alignedRight);

    cv::Mat inlierMask;
    cv::Mat homography;
    if(matches.size() >= 4) {
        homography = findHomography(alignedLeft.points, alignedRight.points,
                                    cv::RANSAC, RANSAC_THRESHOLD, inlierMask);
    }

    if(matches.size() < 4 || homography.empty()) {
        return 0;
    }

    return cv::countNonZero(inlierMask);
}


std::map<float, ImagePair> sortViewsForBaseline() {
    //sort pairwise matches to find the lowest Homography inliers [Snavely07 4.2]
    std::map<float, ImagePair> matchesSizes;
    const size_t numImages = mImages.size();
    for (size_t i = 0; i < numImages - 1; i++) {
        for (size_t j = i + 1; j < numImages; j++) {
            if (mFeatureMatchMatrix[i][j].size() < 100) {
                //Not enough points in matching
                matchesSizes[1.0] = {i, j};
                continue;
            }

            //Find number of homography inliers
            const int numInliers = findHomographyInliers(
                    mImageFeatures[i],
                    mImageFeatures[j],
                    mFeatureMatchMatrix[i][j]);
            const float inliersRatio = (float)numInliers / (float)(mFeatureMatchMatrix[i][j].size());
            matchesSizes[inliersRatio] = {i, j};
        }
    }

    return matchesSizes;
}



bool findCameraMatricesFromMatch(
        const Intrinsics&   intrinsics,
        const Matching2&     matches,
        const Features&     featuresLeft,
        const Features&     featuresRight,
		Matching2&     		prunedMatches,
        cv::Matx34f&        Pleft,
        cv::Matx34f&        Pright) {

    if (intrinsics.K.empty()) {
        std::cerr << "Intrinsics matrix (K) must be initialized." << std::endl;
        return false;
    }

    double focal = intrinsics.K.at<float>(0, 0); //Note: assuming fx = fy
    cv::Point2d pp(intrinsics.K.at<float>(0, 2), intrinsics.K.at<float>(1, 2));

    Features alignedLeft;
    Features alignedRight;
    GetAlignedPointsFromMatch(featuresLeft, featuresRight, matches, alignedLeft, alignedRight);

    cv::Mat E, R, t;
    cv::Mat mask;
    E = cv::findEssentialMat(alignedLeft.points, alignedRight.points, focal, pp, cv::RANSAC, 0.999, 1.0, mask);

    //Find Pright camera matrix from the essential matrix
    //Cheirality check (all points are in front of camera) is performed internally.
    cv::recoverPose(E, alignedLeft.points, alignedRight.points, R, t, focal, pp, mask);


    Pleft = cv::Matx34f::eye();
    Pright = cv::Matx34f(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
                     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2));

    //populate pruned matches
    prunedMatches.clear();
    for (size_t i = 0; i < mask.rows; i++) {
    	if (mask.at<uchar>(i)) {
    		prunedMatches.push_back(matches[i]);
    	}
    }

    return true;
}



bool triangulateViews(
        const Intrinsics&  intrinsics,
        const ImagePair    imagePair,
        const Matching2&    matches,
        const Features&    featuresLeft,
        const Features&    featuresRight,
        const cv::Matx34f& Pleft,
        const cv::Matx34f& Pright,
        PointCloud&        pointCloud) {

    //get aligned features left-right, with back reference to original indexing
    std::vector<int> leftBackReference;
    std::vector<int> rightBackReference;
    Features alignedLeft;
    Features alignedRight;
    GetAlignedPointsFromMatch(
            featuresLeft,
            featuresRight,
            matches,
            alignedLeft,
            alignedRight,
            leftBackReference,
            rightBackReference);

    cv::Mat normalizedLeftPts;
    cv::Mat normalizedRightPts;
    undistortPoints(alignedLeft.points,  normalizedLeftPts,  intrinsics.K, cv::Mat());
    undistortPoints(alignedRight.points, normalizedRightPts, intrinsics.K, cv::Mat());

    cv::Mat points3dHomogeneous;
    triangulatePoints(Pleft, Pright, normalizedLeftPts, normalizedRightPts, points3dHomogeneous);

    cv::Mat points3d;
    convertPointsFromHomogeneous(points3dHomogeneous.t(), points3d);

    cv::Mat rvecLeft;
    Rodrigues(Pleft.get_minor<3, 3>(0, 0), rvecLeft);
    cv::Mat tvecLeft(Pleft.get_minor<3, 1>(0, 3).t());

    std::vector<cv::Point2f> projectedOnLeft(alignedLeft.points.size());
    projectPoints(points3d, rvecLeft, tvecLeft, intrinsics.K, cv::Mat(), projectedOnLeft);

    cv::Mat rvecRight;
    Rodrigues(Pright.get_minor<3, 3>(0, 0), rvecRight);
    cv::Mat tvecRight(Pright.get_minor<3, 1>(0, 3).t());

    std::vector<cv::Point2f> projectedOnRight(alignedRight.points.size());
    projectPoints(points3d, rvecRight, tvecRight, intrinsics.K, cv::Mat(), projectedOnRight);


    //Note: cheirality check (all points z > 0) was already performed at camera pose calculation

    for (size_t i = 0; i < points3d.rows; i++) {
        auto reprojectErrorLeft = cv::norm(projectedOnLeft[i]  - alignedLeft.points[i]);
        auto reprojectErrorRight = cv::norm(projectedOnRight[i] - alignedRight.points[i]);

        if (reprojectErrorLeft  > 10.f or
            reprojectErrorRight > 10.f)
        {
            continue;
        }

        numReprojections++;
        meanReprojectionError += (reprojectErrorLeft - meanReprojectionError) / numReprojections;
        numReprojections++;
        meanReprojectionError += (reprojectErrorRight -  meanReprojectionError) / numReprojections;

        if (reprojectErrorLeft < minReprojectionError)
        {
            minReprojectionError = reprojectErrorLeft;
        }

        if (reprojectErrorRight < minReprojectionError)
        {
            minReprojectionError = reprojectErrorRight;
        }

        if (reprojectErrorLeft > maxReprojectionError)
        {
            maxReprojectionError = reprojectErrorLeft;
        }

        if (reprojectErrorRight > maxReprojectionError)
        {
            maxReprojectionError = reprojectErrorRight;
        }

        std::cout << "--------" << std::endl;
        std::cout << "Actual (left): " << alignedLeft.points[i] << std::endl;
        std::cout << "Reprojected (left): " << projectedOnLeft[i] << std::endl;
        std::cout << "Actual (right): " << alignedRight.points[i] << std::endl;
        std::cout << "Reprojected (right): " << projectedOnRight[i] << std::endl;

        Point3DInMap p;
        p.p = cv::Point3f(points3d.at<float>(i, 0),
                      points3d.at<float>(i, 1),
                      points3d.at<float>(i, 2)
                      );

        //use back reference to point to original features in images
        p.originatingViews[imagePair.left]  = leftBackReference [i];
        p.originatingViews[imagePair.right] = rightBackReference[i];

        pointCloud.push_back(p);
    }

    return true;
}


int main(int argc, char** argv) {


    #pragma region Parse Inputs
    std::string input_image_dir;
    std::string output_dir;
    int opt;
    
    while ((opt = getopt(argc, argv, "i:o:")) != -1) {
        switch (opt) {
            case 'i': {
                input_image_dir = optarg;
                break; 
            }
            case 'o': {
                output_dir = optarg;
                break;
            }
            default: {
                return -1;
            }
        }
    }

    std::vector<std::string> fn;
    cv::glob(input_image_dir + "/*.png", fn, false);
    // TODO: fix format for *.png and *.jpg
    // TODO: error handling if glob fails/image directory is invalid

    std::cout << "Opening images..." << std::endl;


    for (auto& imageFilename : fn) {
        mImages.push_back(cv::imread(imageFilename));

        if (mImages.back().empty()) {
            std::cerr << "Unable to read image from file: " << imageFilename << std::endl;
        }
    }


     //initialize intrinsics
    mIntrinsics.K = (cv::Mat_<float>(3,3) << 2500,   0, mImages[0].cols / 2,
                                           0, 2500, mImages[0].rows / 2,
                                           0,    0, 1);
    mIntrinsics.Kinv = mIntrinsics.K.inv();
    mIntrinsics.distortion = cv::Mat_<float>::zeros(1, 4);

    mCameraPoses.resize(mImages.size());



    std::cout << "----------- Extract features ------------" << std::endl;
#ifdef USE_SIFT
    auto mDetector = cv::SIFT::create();
#else
    auto mDetector = cv::ORB::create(5000);
#endif


    mImageFeatures.resize(mImages.size());
    for (size_t i = 0; i < mImages.size(); i++) {
        Features features;
        mDetector->detectAndCompute(mImages[i], cv::noArray(), features.keyPoints, features.descriptors);

        features.points.clear();
        for (const auto& kp : features.keyPoints) {
            features.points.push_back(kp.pt);
        }

        mImageFeatures[i] = features;

        std::cout << "Image " << i << ": " << mImageFeatures[i].keyPoints.size() << " keypoints" << std::endl;
    }





    std::cout << "----------- Create Feature Match Matrix ------------" << std::endl;

    const size_t numImages = mImages.size();
    mFeatureMatchMatrix.resize(numImages, std::vector<Matching2>(numImages));

    //prepare image pairs to match
    std::vector<ImagePair> pairs;
    for (size_t i = 0; i < numImages; i++) {
        for (size_t j = i + 1; j < numImages; j++) {
            pairs.push_back({ i, j });
        }
    }
    for (int j = 0; j < pairs.size(); j++) {
        const int pairId = j;
        const ImagePair& pair = pairs[pairId];


        //initial matching between features
        std::vector<Matching2> initialMatching;

#ifdef USE_FLANN_BASED
        auto mMatcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
#else
        auto mMatcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
#endif

        mMatcher->knnMatch(mImageFeatures[pair.left].descriptors, mImageFeatures[pair.right].descriptors, initialMatching, 2);

        //prune the matching using the ratio test
        Matching2 prunedMatching;
        for(unsigned i = 0; i < initialMatching.size(); i++) {
            if(initialMatching[i][0].distance < 0.8f * initialMatching[i][1].distance) {
                prunedMatching.push_back(initialMatching[i][0]);
            }
        }

        mFeatureMatchMatrix[pair.left][pair.right] = prunedMatching;
    }



    std::cout << "----------- Find Baseline Triangulation ------------" << std::endl;

    //maps are sorted, so the best pair is at the beginning
    std::map<float, ImagePair> pairsHomographyInliers = sortViewsForBaseline();

    cv::Matx34f Pleft  = cv::Matx34f::eye();
    cv::Matx34f Pright = cv::Matx34f::eye();


    //try to find the best pair, stating at the beginning
    for (auto& imagePair : pairsHomographyInliers) {
        size_t i = imagePair.second.left;
        size_t j = imagePair.second.right;


        Matching2 prunedMatching;
        //recover camera matrices (poses) from the point matching
        auto success = findCameraMatricesFromMatch(
                mIntrinsics,
                mFeatureMatchMatrix[i][j],
                mImageFeatures[i],
                mImageFeatures[j],
				prunedMatching,
                Pleft, Pright
                );

        if (not success) {
            continue;
        }

        float poseInliersRatio = (float)prunedMatching.size() / (float)mFeatureMatchMatrix[i][j].size();

        if (poseInliersRatio < POSE_INLIERS_MINIMAL_RATIO) {
            continue;
        }

        mFeatureMatchMatrix[i][j] = prunedMatching;

        PointCloud pointCloud;
        success = triangulateViews(
                mIntrinsics,
                imagePair.second,
                mFeatureMatchMatrix[i][j],
                mImageFeatures[i], mImageFeatures[j],
                Pleft, Pright,
                pointCloud
                );

       if (not success) {
           continue;
       }

       mReconstructionCloud = pointCloud;
       mCameraPoses[i] = Pleft;
       mCameraPoses[j] = Pright;
       mDoneViews.insert(i);
       mDoneViews.insert(j);
       mGoodViews.insert(i);
       mGoodViews.insert(j);


       break;
    }


    std::cout << "Reconstruction complete" << std::endl;
    std::cout << "Number of points used: " << numReprojections/2 << std::endl;
    std::cout << "Mean reprojection error: " << meanReprojectionError << std::endl;
    std::cout << "Min reprojection error: " << minReprojectionError << std::endl;
    std::cout << "Max reprojection error: " << maxReprojectionError << std::endl;



    std::cout << "Saving reconstructed point cloud and camera poses to PLY" << std::endl;

    std::ofstream ofs("points.ply");

    //write PLY header
    ofs << "ply                 " << std::endl <<
           "format ascii 1.0    " << std::endl <<
           "element vertex " << mReconstructionCloud.size() << std::endl <<
           "property float x    " << std::endl <<
           "property float y    " << std::endl <<
           "property float z    " << std::endl <<
           "property uchar red  " << std::endl <<
           "property uchar green" << std::endl <<
           "property uchar blue " << std::endl <<
           "end_header          " << std::endl;

    for (const Point3DInMap& p : mReconstructionCloud) {
    	//get color from first originating view
		auto originatingView = p.originatingViews.begin();
		const int viewIdx = originatingView->first;
		cv::Point2f p2d = mImageFeatures[viewIdx].points[originatingView->second];
		cv::Vec3b pointColor = mImages[viewIdx].at<cv::Vec3b>(p2d);

		//write vertex
        ofs << p.p.x              << " " <<
        	   p.p.y              << " " <<
			   p.p.z              << " " <<
			   (int)pointColor(2) << " " <<
			   (int)pointColor(1) << " " <<
			   (int)pointColor(0) << " " << std::endl;
    }

    ofs.close();

    std::ofstream ofsc("cameras.ply");

    //write PLY header
    ofsc << "ply                 " << std::endl <<
           "format ascii 1.0    " << std::endl <<
           "element vertex " << (mCameraPoses.size() * 4) << std::endl <<
           "property float x    " << std::endl <<
           "property float y    " << std::endl <<
           "property float z    " << std::endl <<
           "element edge " << (mCameraPoses.size() * 3) << std::endl <<
           "property int vertex1" << std::endl <<
           "property int vertex2" << std::endl <<
           "property uchar red  " << std::endl <<
           "property uchar green" << std::endl <<
           "property uchar blue " << std::endl <<
           "end_header          " << std::endl;

    //save cameras polygons..
    for (const auto& pose : mCameraPoses) {
        cv::Point3d c(pose(0, 3), pose(1, 3), pose(2, 3));
        cv::Point3d cx = c + cv::Point3d(pose(0, 0), pose(1, 0), pose(2, 0)) * 0.2;
        cv::Point3d cy = c + cv::Point3d(pose(0, 1), pose(1, 1), pose(2, 1)) * 0.2;
        cv::Point3d cz = c + cv::Point3d(pose(0, 2), pose(1, 2), pose(2, 2)) * 0.2;

        ofsc << c.x  << " " << c.y  << " " << c.z  << std::endl;
        ofsc << cx.x << " " << cx.y << " " << cx.z << std::endl;
        ofsc << cy.x << " " << cy.y << " " << cy.z << std::endl;
        ofsc << cz.x << " " << cz.y << " " << cz.z << std::endl;
    }

    const int camVertexStartIndex = mReconstructionCloud.size();

    for (size_t i = 0; i < mCameraPoses.size(); i++) {
        ofsc << (i * 4 + 0) << " " <<
                (i * 4 + 1) << " " <<
                "255 0 0" << std::endl;
        ofsc << (i * 4 + 0) << " " <<
                (i * 4 + 2) << " " <<
                "0 255 0" << std::endl;
        ofsc << (i * 4 + 0) << " " <<
                (i * 4 + 3) << " " <<
                "0 0 255" << std::endl;
    }



    return 0;
}
