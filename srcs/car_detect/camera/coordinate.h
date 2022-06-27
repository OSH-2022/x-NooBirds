#ifndef COORDINATE_H_
#define COORDINATE_H_

#include "includes.h"

using namespace cv;

class Coordinate {
public:
    Coordinate(int id);
    ~Coordinate();

private:
    VideoCapture dev;
    int basePoint[4] = {-1, -1, -1, -1};
};

#endif