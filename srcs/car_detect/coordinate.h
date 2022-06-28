#ifndef COORDINATE_H_
#define COORDINATE_H_

#include "includes.h"
#include "timer.h"

typedef struct DataPackage {
    AcrtTime time;
    Point2i cars[3];
    vector<Point> dangerObj;
} Package;

class Coordinate {
public:
    Coordinate(int id);
    ~Coordinate();

    const mutex& getMutex();
    const Package& getData();
    
    void run();

private:
    bool updateBase(const Mat &erodeHsv);
    bool trackObject(const Mat &frame, const Mat &realHsv, const AcrtTime &now);

    int count;

    VideoCapture cap;
    Mat convert;

    mutex dataMutex;
    Package data;
};

#endif