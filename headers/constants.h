//
// Created by Megan Finch on 13/12/2021.
//

#ifndef SFM_CONSTANTS_H
#define SFM_CONSTANTS_H

const float LOWE_RATIO = 0.6f;

// The number of matches required between two images for them to be connected in the scene graph
const int MATCH_COUNT_THRESHOLD = 25;

const bool DEBUG_MODE = true;

const bool USE_CV_SFM_TRIANGULATION = true;

typedef int ImageID;
typedef int KeyPointIDX;

#endif //SFM_CONSTANTS_H
