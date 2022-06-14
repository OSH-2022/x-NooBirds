#ifndef DETECT_H_
#define DETECT_H_

#include "includes.h"

enum color_set {red, yellow};

#define DETECT_RANGE 3

#define RED_MIN 160
#define RED_DELTA 140

#define POINT_COLOR red

class Pair {
public:
    Pair();
    Pair(float _x, float _y);
    Pair& operator=(const Pair &tar);

    float x;
    float y;
};

class Space {
public:
    Space(const uint8_t* yuyv, unsigned int _width, unsigned int _height);
    ~Space();

    Pair photo2real(const Pair &photo);

    void update(const uint8_t* yuyv);
    Pair track(const uint8_t* yuyv, color_set color);

private:
    int width;
    int height;
    Pair center;
    Pair point_left;
    Pair point_right;
    Pair point_far;
    Pair point_near;
};

#endif // DETECT_H_