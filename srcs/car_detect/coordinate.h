#ifndef COORDINATE_H_
#define COORDINATE_H_

#include "includes.h"
#include "timer.h"

using namespace std;
using namespace cv;

typedef struct DataPackage {
    AcrtTime time;
    Point cars[3];
} Package;

class Coordinate {
public:
    Coordinate(int id);
    ~Coordinate();

    const mutex& getMutex();
    const Package& getData();
    
    bool run();

private:
    bool updateBase(const Mat &erodeHsv);
    bool trackObject(const Mat &erodeHsv, const AcrtTime &now);

    int count;

    VideoCapture cap;
    Mat convert;

    mutex dataMutex;
    Package data;
};

#endif