#include "timer.h"

void get_time(accurate_time_t *t) {
    struct timeval time;
    gettimeofday(&time, NULL);

    t->min = time.tv_sec / 60 % 60;
    t->sec = time.tv_sec % 60;
    t->msec = time.tv_usec / 1000;
}

void print_time() {
    struct timeval time;
    gettimeofday(&time, NULL);

    printf(
        "min: %2ld, sec: %2ld, msec: %3ld\n",
        time.tv_sec / 60 % 60,
        time.tv_sec % 60,
        time.tv_usec / 1000
    );
}