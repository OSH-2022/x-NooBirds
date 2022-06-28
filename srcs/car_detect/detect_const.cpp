#include "includes.h"
#include "detect_const.h"

Mat isRed(const Mat &src) {
    Mat range1, range2;
    inRange(src, RED_LOWER_LIMIT1, RED_UPPER_LIMIT1, range1);
    inRange(src, RED_LOWER_LIMIT2, RED_UPPER_LIMIT2, range2);
    return range1 + range2;
}

Mat isYellow(const Mat &src) {
    Mat range;
    inRange(src, YELLOW_LOWER_LIMIT, YELLOW_UPPER_LIMIT, range);
    return range;
}

Mat isGreen(const Mat &src) {
    Mat range;
    inRange(src, GREEN_LOWER_LIMIT, GREEN_UPPER_LIMIT, range);
    return range;
}

Mat isBlue(const Mat &src) {
    Mat range;
    inRange(src, BLUE_LOWER_LIMIT, BLUE_UPPER_LIMIT, range);
    return range;
}

Mat isColor(const Mat &src, color_t color) {
    switch (color)
    {
    case    red: return isRed(src);
    case yellow: return isYellow(src);
    case  green: return isGreen(src);
    case   blue: return isBlue(src);
    default: return Mat();
    }
}