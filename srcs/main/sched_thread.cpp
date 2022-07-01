#include "sched_thread.h"

RVOScheduler::RVOScheduler() {
    this->sim = new RVO::RVOSimulator();
    sim->setTimeStep(0.05f);
    // setup agents TODO
    sim->setAgentDefaults(300.0f, 1, 10.0f, 10.0f, 300.0f, 10.0f);
    for (int i = 0; i < 3; i++) {
        sim->addAgent(RVO::Vector2(i * 200, 0));
        goals.push_back(RVO::Vector2(0, 0));
    }
}

mutex &RVOScheduler::getNewDataMutex() {
    return this->newDataMutex;
}

void RVOScheduler::step() {
    for (int i = 0; i < 3; ++i) {
        sim->setAgentPosition(i, data.cars[i]);
        sim->setAgentVelocity(i, data.vels[i]);

		RVO::Vector2 goalVector = goals[i] - data.cars[i];
		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector);
		}
		sim->setAgentPrefVelocity(i, goalVector);
	}

    sim->doStep();

    {
        lock_guard<mutex> guard(newDataMutex);
        newData.time.update();
        for (int i = 0; i < 3; ++i) {
            newData.cars[i] = sim->getAgentPosition(i);
            newData.vels[i] = sim->getAgentVelocity(i);
        }
    }
}

void RVOScheduler::setData(const PackageWithVel &d) {
    data = d;
}

void RVOScheduler::setGoal(const std::vector<RVO::Vector2> &g) {
    goals = g;
}

const PackageWithVel &RVOScheduler::getNewData() {
    lock_guard<mutex> guard(newDataMutex);
    return this->newData;
}

RVOScheduler::~RVOScheduler() {
    ;
}