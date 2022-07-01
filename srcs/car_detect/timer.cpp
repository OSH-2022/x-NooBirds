#include "timer.h"

AcrtTime::AcrtTime() {
    struct timeval time;
    gettimeofday(&time, NULL);

    min = time.tv_sec / 60 % 60;
    sec = time.tv_sec % 60;
    msec = time.tv_usec / 1000;
}

AcrtTime::AcrtTime(int min_, int sec_, int msec_) {
    min = min_;
    sec = sec_;
    msec = msec_;
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

int AcrtTime::operator-(const AcrtTime &time) const {
    int rst = (min - time.min) * 60000
            + (sec - time.sec) * 1000
            + msec - time.msec;
    if (min < time.min) rst += 3600000;
    return rst;
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