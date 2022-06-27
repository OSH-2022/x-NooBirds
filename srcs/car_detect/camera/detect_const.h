#ifndef DETECT_CONST_H_
#define DETECT_CONST_H_

#include "includes.h"

using namespace std;
using namespace cv;

#define FRAME_WIDTH  1920
#define FRAME_HEIGHT 1080

#define RED_LOWER_LIMIT1 Scalar(170, 100, 100)
#define RED_UPPER_LIMIT1 Scalar(180, 255, 255)
#define RED_LOWER_LIMIT2 Scalar(  0, 100, 100)
#define RED_UPPER_LIMIT2 Scalar( 10, 255, 255)

#define REAL 400
Point2f realPoints[4] = {
    Point2f(   0,   0), Point2f(   0, REAL),
    Point2f(REAL,   0), Point2f(REAL, REAL)
};

Mat isRed(const Mat &src);

#endif