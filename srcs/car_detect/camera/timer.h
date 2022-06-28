#ifndef TIMER_H_
#define TIMER_H_

#include "includes.h"

class AcrtTime {
public:
    int min;
    int sec;
    int msec;

    AcrtTime();
    ~AcrtTime();

    void update();

    AcrtTime &operator=(const AcrtTime &time);
};

void printTime();

#endif // TIMER_H_