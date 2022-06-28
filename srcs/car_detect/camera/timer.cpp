#include "timer.h"

AcrtTime::AcrtTime() {
    struct timeval time;
    gettimeofday(&time, NULL);

    min = time.tv_sec / 60 % 60;
    sec = time.tv_sec % 60;
    msec = time.tv_usec / 1000;
}

AcrtTime::~AcrtTime() { }

void AcrtTime::update() {
    struct timeval time;
    gettimeofday(&time, NULL);

    min = time.tv_sec / 60 % 60;
    sec = time.tv_sec % 60;
    msec = time.tv_usec / 1000;
}

AcrtTime& AcrtTime::operator=(const AcrtTime &time) {
    min = time.min;
    sec = time.sec;
    msec = time.msec;
    return *this;
}

void printTime() {
    struct timeval time;
    gettimeofday(&time, NULL);

    printf(
        "min: %2ld, sec: %2ld, msec: %3ld\n",
        time.tv_sec / 60 % 60,
        time.tv_sec % 60,
        time.tv_usec / 1000
    );
}