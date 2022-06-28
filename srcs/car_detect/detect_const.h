#ifndef DETECT_CONST_H_
#define DETECT_CONST_H_

#include "includes.h"

using namespace std;
using namespace cv;

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define FPS 30

#define RED_LOWER_LIMIT1   Scalar(170, 100, 100)
#define RED_UPPER_LIMIT1   Scalar(180, 255, 255)
#define RED_LOWER_LIMIT2   Scalar(  0, 100, 100)
#define RED_UPPER_LIMIT2   Scalar( 10, 255, 255)

#define ORANGE_LOWER_LIMIT Scalar( 10, 100,  50)
#define ORANGE_UPPER_LIMIT Scalar( 20, 255, 255)

#define YELLOW_LOWER_LIMIT Scalar( 20, 100,  50)
#define YELLOW_UPPER_LIMIT Scalar( 35, 255, 255)

#define BLUE_LOWER_LIMIT   Scalar( 80,  50,  50)
#define BLUE_UPPER_LIMIT   Scalar(100, 255, 255)

#define REAL 400

#define DETECT_TOLERANCE 10
#define OBJ_MIN_AREA 36

enum color_t {red, orange, yellow, blue};

Mat isRed(const Mat &src);
Mat isOrange(const Mat &src);
Mat isYellow(const Mat &src);
Mat isBlue(const Mat &src);

Mat isColor(const Mat &src, color_t color);

#endif