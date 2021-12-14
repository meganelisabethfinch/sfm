#include "image.hpp"
#include "feature_matcher.hpp"

#include <string>
#include <unistd.h>
#include <iostream>
#include <filesystem>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <triangulator.hpp>
// #include <opencv2/sfm/triangulation.hpp>

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
    std::vector<Image> input_images;
    for (int i = 0; i < fn.size(); i++) {
        std::string name = fs::path(fn[i]).filename();
        cv::Mat img = imread(fn[i], IMREAD_COLOR);

        input_images.push_back(Image(i, name, img));
    }
    #pragma endregion Parse Inputs

    #pragma region Feature Detection
    std::cout << "Finding features..." << std::endl;

    FeatureMatcher matcher;

    matcher.detect(input_images);

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

    matcher.match(input_images);

    matcher.getSceneGraph(input_images);
    #pragma endregion Feature Matching

    // 4. image registration

    // 5. triangulation
    std::cout << "Triangulating points..." << std::endl;
    Triangulator triangulator;

    triangulator.reconstruct(input_images);

    // 6. bundle adjustment

    return 0;
}