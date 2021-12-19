//
// Created by Megan Finch on 19/12/2021.
//

#ifndef SFM_SFM_UTILITIES_HPP
#define SFM_SFM_UTILITIES_HPP

#include "image.hpp"

class SFMUtilities {
public:
/**
 *
 * @param image1
 * @param image2
 * @return the number of homography inliers between the two images
 */
static int findHomographyInliers(Image &image1, Image &image2);

};
#endif //SFM_SFM_UTILITIES_HPP
