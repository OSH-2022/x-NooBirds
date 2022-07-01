#ifndef TIMER_H_
#define TIMER_H_

#include "includes.h"

class AcrtTime {
public:
    int min;
    int sec;
    int msec;

    AcrtTime();
    AcrtTime(int min_, int sec_, int msec_);
    ~AcrtTime();

    void update();

    AcrtTime &operator=(const AcrtTime &time);
    int operator-(const AcrtTime &time) const;
};

void printTime();

#endif // TIMER_H_