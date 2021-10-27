#include "image.hpp"

#include <string>
#include <unistd.h>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>

using namespace cv;

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
        // all_keypoints[i] = keypoints;
        // all_descriptors[i] = descriptors;
    }

    Mat out;
    drawKeypoints(input_images[0].img, input_images[0].keypoints, out, flags = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    #pragma endregion Feature Detection

    #pragma region Feature Matching
    std::cout << "Finding matches..." << std::endl;

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    #pragma endregion Feature Matching

    // 3. geometric verification

    // 4. image registration

    // 5. triangulation

    // 6. bundle adjustment

    return 0;
}