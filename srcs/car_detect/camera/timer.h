#ifndef TIMER_H_
#define TIMER_H_

#include "includes.h"

typedef struct {
    int min;
    int sec;
    int msec;
} accurate_time_t;

void get_time(accurate_time_t *t);

#endif // TIMER_H_