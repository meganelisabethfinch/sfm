#include "image.hpp"
#include "feature_matcher.hpp"

#include <string>
#include <unistd.h>
#include <iostream>
#include <filesystem>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/sfm/triangulation.hpp>

using namespace cv;
namespace fs = std::filesystem;

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

    std::vector<String> fn;
    glob(input_image_dir + "/*.png", fn, false);
    // TODO: fix format for *.png and *.jpg
    // TODO: error handling if glob fails/image directory is invalid

    std::cout << "Opening images..." << std::endl;
    std::vector<Image> input_images(fn.size());
    for (size_t i = 0; i < fn.size(); i++) {
        input_images[i].name = fs::path(fn[i]).filename();
        input_images[i].img = imread(fn[i], IMREAD_COLOR);
    }
    #pragma endregion Parse Inputs

    #pragma region Feature Detection
    std::cout << "Finding features..." << std::endl;

    Ptr<SIFT> detector = SIFT::create();
    std::vector<std::vector<KeyPoint>> all_keypoints(input_images.size());
    std::vector<Mat> all_descriptors(input_images.size());
    for (size_t i = 0; i < input_images.size(); i++) {
        std::vector<KeyPoint> keypoints;
        Mat descriptors;

        detector->detectAndCompute(input_images[i].img, noArray(), input_images[i].keypoints, input_images[i].descriptors);
    }

    // Output example keypoints for debug
    /*
    Mat out;
    drawKeypoints(input_images[0].img, input_images[0].keypoints, out, Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("Keypoints", out);
    cv::waitKey(0);
    cv::destroyWindow("Keypoints");
    */

    #pragma endregion Feature Detection

    #pragma region Feature Matching and Geometric Verification
    std::cout << "Finding matches..." << std::endl;

    FeatureMatcher matcher(0.6);
    matcher.match(input_images);

    matcher.getSceneGraph(input_images);
    #pragma endregion Feature Matching

    // 4. image registration

    // 5. triangulation
    // triangulatePoints()

    // 6. bundle adjustment

    return 0;
}