#include "includes.h"
#include "detect_const.h"

#define RED_LOWER_LIMIT1 Scalar(170, 100, 100)
#define RED_UPPER_LIMIT1 Scalar(180, 255, 255)
#define RED_LOWER_LIMIT2 Scalar(  0, 100, 100)
#define RED_UPPER_LIMIT2 Scalar( 10, 255, 255)

Mat isRed(const Mat &src) {
    Mat range1, range2;
    inRange(src, RED_LOWER_LIMIT1, RED_UPPER_LIMIT1, range1);
    inRange(src, RED_LOWER_LIMIT2, RED_UPPER_LIMIT2, range2);
    return range1 + range2;
}