#ifndef COORDINATE_H_
#define COORDINATE_H_

#include "includes.h"
#include "timer.h"

using namespace std;
using namespace cv;

typedef struct DataPackage {
    AcrtTime time;
    Point cars[3];
    vector<pair<Point, float>> dangerObj;
} Package;

class Coordinate {
public:
    Coordinate(int id);
    ~Coordinate();

    mutex& getMutex();
    const Package& getData();

    bool run(bool fake=false);

private:
    bool updateBase(const Mat &hsv);
    bool trackObject(const Mat &hsv, const AcrtTime &now, const Mat &rgb);

    int count;

    VideoCapture cap;
    Mat convert;
    Ptr<BackgroundSubtractorKNN> fgbg;

    mutex dataMutex;
    Package data;
};

#endif