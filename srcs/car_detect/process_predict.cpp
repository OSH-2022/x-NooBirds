#include "includes.h"
#include "process_predict.h"

Predict::Predict() { paraVld = false; }

Predict::~Predict() { }

void Predict::push(const AcrtTime &time, double value) {
    if (past.empty()) {
        for (int i = 0; i < HISTORY_LENGTH; i++) {
            past.emplace_back(time);
            process.emplace_back(value);
            a = b = 0;
            paraVld = true;
        }
    } else {
        past.erase(past.begin());
        process.erase(process.begin());
        past.emplace_back(time);
        process.emplace_back(value);
    }
}

void Predict::updatePara() {
    double lastValue = process.back();
    AcrtTime lastTime = past.back();
    int msecSum = 0;
    int squareSum = 0;
    double valueSum = 0;
    double multiSum = 0;
    auto itr1 = past.begin();
    auto itr2 = process.begin();
    while (itr1 != past.end()) {
        int deltaTime = lastTime - *itr1;
        double deltaValue = lastValue - *itr2;
        msecSum += deltaTime;
        valueSum += deltaValue;
        squareSum += deltaTime * deltaTime;
        multiSum = deltaTime * deltaValue;
        itr1++;
        itr2++;
    }
    double avgMsec = msecSum / HISTORY_LENGTH;
    double avgValue = valueSum / HISTORY_LENGTH;
    double base = squareSum - avgMsec * avgMsec * HISTORY_LENGTH;
    if (base < 1e-8) a = b = 0;
    else {
        a = multiSum - (avgMsec * avgValue * HISTORY_LENGTH) / base;
        b = avgValue - avgMsec * a;
    }
    paraVld = true;
}

double Predict::predict(const AcrtTime &future) {
    if (!paraVld) updatePara();
    int msec = future - past.back();
    return process.back() + msec * a + b;
}