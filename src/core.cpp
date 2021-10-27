#include <string>
#include <unistd.h>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>

int main(int argc, char** argv) {
    #pragma region Parse_Command_Line
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

    std::vector<cv::String> fn;
    cv::glob(input_image_dir + "/*.png", fn, false);
    // TODO: fix format for *.png and *.jpg
    // TODO: error handling if glob fails/image directory is invalid

    std::cout << "Opening images..." << std::endl;
    std::vector<cv::Mat> input_images;
    for (size_t i = 0; i < fn.size(); i++) {
        input_images.push_back(cv::imread(fn[i], cv::IMREAD_COLOR));
    }
    #pragma endregion Parse_Command_Line

    #pragma region Feature_Detection
    std::cout << "Finding features..." << std::endl;

    cv::Ptr<cv::SIFT> detector = cv::SIFT::create();
    std::vector<std::vector<cv::KeyPoint>> all_keypoints;
    std::vector<cv::Mat> all_descriptors;
    for (size_t i = 0; i < input_images.size(); i++) {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        detector->detectAndCompute(input_images[i], cv::noArray(), keypoints, descriptors);
        all_keypoints.push_back(keypoints);
        all_descriptors.push_back(descriptors);
    }

    #pragma endregion Feature_Detection

    // 2. feature matching

    // 3. geometric verification

    // 4. image registration

    // 5. triangulation

    // 6. bundle adjustment

    return 0;
}