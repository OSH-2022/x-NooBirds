#include "includes.h"
#include "process_predict.h"
#include <iostream>
Predict::Predict() {
    lastVld = false;
    paraVld = false;
}

Predict::~Predict() { }

void Predict::push(const AcrtTime &time, double value) {
    paraVld = false;
    if (lastVld) {
        if (time - lastTime == 0) return;
        if (midTime.size() >= HISTORY_LENGTH) {
            midTime.erase(midTime.begin());
            k.erase(k.begin());
        }
        midTime.emplace_back(lastTime.mid(time));
        k.emplace_back((value - lastValue) / (time - lastTime));
        lastTime = time;
        lastValue = value;
    } else {
        lastVld = true;
        lastTime = time;
        lastValue = value;
    }
}

void Predict::updatePara() {
    if (midTime.size() == 0) return;
    AcrtTime lastMidTime = midTime.back();
    double msecSum = 0;
    double squareSum = 0;
    double kSum = 0;
    double multiSum = 0;
    auto itr1 = midTime.begin();
    auto itr2 = k.begin();
    while (itr1 != midTime.end()) {
        int deltaTime = lastMidTime - *itr1;
        msecSum += deltaTime;
        kSum += *itr2;
        squareSum += deltaTime * deltaTime;
        multiSum += deltaTime * *itr2;
        itr1++;
        itr2++;
    }
    double avgMsec = msecSum / midTime.size();
    double avgK = kSum / midTime.size();
    double base = squareSum - avgMsec * avgMsec * midTime.size();
    if (base < 1e-8 && base > -1e-8) paraVld = false;
    else {
        a = (multiSum - avgMsec * avgK * midTime.size()) / base;
        b = avgK - avgMsec * a;
        paraVld = true;
    }
}

double Predict::predict(const AcrtTime &future) {
    return (future - lastTime) * predictK(lastTime.mid(future)) + lastValue;
}

double Predict::predictK(const AcrtTime &future) {
    if (!paraVld)
        updatePara();
    float msec = future - midTime.back();
    return b - msec * a;
}