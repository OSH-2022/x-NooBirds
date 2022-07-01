#ifndef PROCESS_PREDICT_H_
#define PROCESS_PREDICT_H_

#include "includes.h"
#include "timer.h"

#define HISTORY_LENGTH 5

using namespace std;

class Predict {
public:
    Predict();
    void push(const AcrtTime &time, double value);
    void updatePara();
    double predict(const AcrtTime &future);
    ~Predict();

private:
    list<AcrtTime> midTime;
    list<double> k;

    bool lastVld;
    AcrtTime lastTime;
    double lastValue;

    bool paraVld;
    double a, b;
};

#endif