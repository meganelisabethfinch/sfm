#include <string>
#include <unistd.h>
#include <iostream>

#include <opencv2/imgcodecs.hpp>

int main(int argc, char** argv) {
    // parse inputs
    std::string inputImageDir;
    std::string outputDir;
    int opt;
    
    while ((opt = getopt(argc, argv, "i:o:")) != -1) {
        switch (opt) {
            case 'i': {
                inputImageDir = optarg;
                break; 
            }
            case 'o': {
                outputDir = optarg;
                break;
            }
            default: {
                return -1;
            }
        }
    }

    std::vector<cv::String> fn;
    cv::glob(inputImageDir + "/*.png", fn, false);
    // TODO: fix format for *.png and *.jpg
    // TODO: error handling if glob fails/image directory is invalid

    std::vector<cv::Mat> inputImages;
    for (size_t i = 0; i < fn.size(); i++) {
        cv::Mat image = cv::imread(fn[i], 1);
        if (image.data) {
            inputImages.push_back(image);
        }
    }

    // 1. detect features

    // 2. feature matching

    // 3. geometric verification

    // 4. image registration

    // 5. triangulation

    // 6. bundle adjustment

    return 0;
}