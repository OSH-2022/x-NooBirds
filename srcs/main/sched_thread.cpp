#include "sched_thread.h"

RVOScheduler::RVOScheduler() {
    this->sim = new RVO::RVOSimulator();
    sim->setTimeStep(0.05f);

    // Setup agents for RVO2 algorithm to work
    sim->setAgentDefaults(50.0f, 1, 0.1f, 0.1f, 50.0f, 160.0f);

    for (int i = 0; i < 3; i++) {
        sim->addAgent(RVO::Vector2(i * 200, 0));
        goals.push_back(RVO::Vector2(0, 0));
    }

    // add walls around the place
    std::vector<RVO::Vector2> vertices;
    vertices.push_back(RVO::Vector2(30.0f, -30.0f));
    vertices.push_back(RVO::Vector2(30.0f, 430.0f));
    vertices.push_back(RVO::Vector2(-30.0f, 430.0f));
    vertices.push_back(RVO::Vector2(-30.0f, -30.0f));
    sim->addObstacle(vertices);

    vertices.clear();
    vertices.push_back(RVO::Vector2(430.0f, -30.0f));
    vertices.push_back(RVO::Vector2(430.0f, 430.0f));
    vertices.push_back(RVO::Vector2(370.0f, 430.0f));
    vertices.push_back(RVO::Vector2(370.0f, -30.0f));
    sim->addObstacle(vertices);

    vertices.clear();
    vertices.push_back(RVO::Vector2(-30.0f, -30.0f));
    vertices.push_back(RVO::Vector2(430.0f, -30.0f));
    vertices.push_back(RVO::Vector2(430.0f, 30.0f));
    vertices.push_back(RVO::Vector2(-30.0f, 30.0f));
    sim->addObstacle(vertices);

    vertices.clear();
    vertices.push_back(RVO::Vector2(-30.0f, 370.0f));
    vertices.push_back(RVO::Vector2(430.0f, 370.0f));
    vertices.push_back(RVO::Vector2(430.0f, 430.0f));
    vertices.push_back(RVO::Vector2(-30.0f, 430.0f));
    sim->addObstacle(vertices);

    sim->processObstacles();
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

HeuristicScheduler::HeuristicScheduler() {
    ;
}

HeuristicScheduler::~HeuristicScheduler() {
    ;
}

mutex &HeuristicScheduler::getNewDataMutex() {
    return this->newDataMutex;
}

void HeuristicScheduler::setData(const PackageWithVel &d) {
    data = d;
}

const PackageWithVel &HeuristicScheduler::getNewData() {
    lock_guard<mutex> guard(newDataMutex);
    return this->newData;
}

void HeuristicScheduler::schedule() {
    long double x_tmp[OBJ_NUM] = {0.0};
    long double y_tmp[OBJ_NUM] = {0.0};
    long double vx_tmp[OBJ_NUM] = {0.0};
    long double vy_tmp[OBJ_NUM] = {0.0};
    long double adj_coe = 1.0 - adj_step_size;
    int adjusted_car = 0;
    int updated_car = 0;
    int is_safe = 1;
    int i = 0;
    int j = 0;
    int k = 0;
    int l = 0;
    int need_adjust = 0;
    int need_update = 0;
    int ret = 0;
    long double vel_angle[i] = {0.0};

    for (i = 0; i < OBJ_NUM; i++) {
        x_tmp[i] = this->data.cars[i].x();
        y_tmp[i] = this->data.cars[i].y();
        vx_tmp[i] = this->data.vels[i].x();
        vy_tmp[i] = this->data.vels[i].y();
    }

    for (i = 0; i < OBJ_NUM; i++) {
        if (vx_tmp[i] > 0) {
            angle[i] = atan(vy_tmp[i] / vx_tmp[i]);
        }
        else if (vy_tmp[i] > 0) {
            angle[i] = pi + atan(vy_tmp[i] / vx_tmp[i]);
        }
    }

    do {
        need_adjust = 0;

        ret = 0;
        // initial safety detection
        for (j = 0; j < OBJ_NUM - 1; j++) {
            for (k = j + 1; k < OBJ_NUM; k++) {
                long double coe = safety_coe;
                if (corner_case[j][k] == 1) {
                    ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
                    if (ret == 0) {
                        corner_case[j][k] = 0;
                    }
                    else {
                        coe = 1.0;
                    }
                }
                ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], coe);
                if (ret == 1) {
                    need_adjust = 1;
                    break;
                }
            }
            if (ret == 1) {
                break;
            }
        }

        if (ret == 1) {
            int success = 0;
            adj_coe = 1.0 - adj_step_size;
            for (i = 0; i < adj_times; i++) {
                ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
                if (ret == 0) {
                    success = 1;
                    adjusted_car = j;
                    is_adjust[j] = 1;
                    break;
                }
                else {
                    adj_coe -= adj_step_size;
                }
            }

            if (success == 0) {
                adj_coe = 1.0 - adj_step_size;
                for (i = 0; i < adj_times; i++) {
                    ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], adj_coe * vx_tmp[k], adj_coe * vy_tmp[k], safety_coe);
                    if (ret == 0) {
                        success = 1;
                        adjusted_car = k;
                        is_adjust[k] = 1;
                        break;
                    }
                    else {
                        adj_coe -= adj_step_size;
                    }
                }
            }

            if (success == 0) {
                adj_coe = 0.001;
                long double velocity_j = pow(vx_tmp[j], 2) + pow(vy_tmp[j], 2);
                long double velocity_k = pow(vx_tmp[k], 2) + pow(vy_tmp[k], 2);
                corner_case[j][k] = 1;
                long double coe;
                coe = 1.0;
                ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], adj_coe * vx_tmp[k], adj_coe * vy_tmp[k], coe);
                if (ret == 0) {
                    adjusted_car = k;
                    is_adjust[k] = 1;
                    slow_car[j][k] = j;
                }
                else {
                    ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], coe);
                    if (ret == 0) {
                        adjusted_car = j;
                        is_adjust[j] = 1;
                        slow_car[j][k] = k;
                    }
                    else { // TO BE DONE

                    }
                }
            }
        }

        if (need_adjust == 1) {
            for (i = 0; i < OBJ_NUM; i++) {
                if (i == adjusted_car) {
                    vx_tmp[i] = adj_coe * vx_tmp[i];
                    vy_tmp[i] = adj_coe * vy_tmp[i];
                }
            }
        }
    }
    while (need_adjust == 1);

    long double vx_save;
    long double vy_save;
    do
    {
        need_update = 0;
        for (l = 0; l < OBJ_NUM; l++) {
            if (is_adjust[l] == 0) {
                continue;
            }
            else {
                for (i = 0; i < OBJ_NUM; i++) {
                    if (i == l) {
                        long double vel_angle;
                        vel_angle = this->vel_angle[i];
                        vx_save = vx_tmp[i];
                        vy_save = vy_tmp[i];
                        vx_tmp[i] = initial_vel[i] * cos(vel_angle);
                        vy_tmp[i] = initial_vel[i] * sin(vel_angle);
                    }
                }

                ret = 0;
                for (j = 0; j < OBJ_NUM - 1; j++) {
                    for (k = j + 1; k < OBJ_NUM; k++) {
                        long double coe;
                        if (corner_case[j][k] == 1 && ((j != l && k != l) || (l == slow_car[j][k]))) {
                            coe = 1.0;
                        }
                        else {
                            coe = safety_coe * 2.0;
                        }
                        ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], coe);
                        if (ret == 1) {
                            break;
                        }
                    }
                    if (ret == 1) {
                        break;
                    }
                }

                if (ret == 0) {
                    need_update = 1;
                    is_adjust[l] = 0;
                    break;
                }
                else {
                    vx_tmp[l] = vx_save;
                    vy_tmp[l] = vy_save;
                }
            }
        }
    }
    while (need_update == 1);

    for (i = 0; i < OBJ_NUM; i++) {
        this->newData.vels[i] = RVO::Vector2(vx_tmp[i], vy_tmp[i]);
    }
}

int HeuristicScheduler::unsafe_state_detector(int j, int k, long double jx, long double jy, long double jvx, long double jvy, long double kx, long double ky, long double kvx, long double kvy, long double coe) {
    long double vx_rel = jvx - kvx;
    long double vy_rel = jvy - kvy;
    long double x_rel = jx - kx;
    long double y_rel = jy - ky;
    long double x_rel_fin = (jx + jvx * SCHE_INTERVAL) - (kx + kvx * SCHE_INTERVAL);
    long double y_rel_fin = (jy + jvy * SCHE_INTERVAL) - (ky + kvy * SCHE_INTERVAL);
    long double dist_now = pow(x_rel, 2) + pow(y_rel, 2);
    long double dist_fin = pow(x_rel_fin, 2) + pow(y_rel_fin, 2);
    long double a = 0.0;
    long double b = 0.0;
    long double c = 0.0;
    long double min_dist = 0.0;
    long double minimum_dist = 0.0;
    long double dist_now_fin_min = (dist_now > dist_fin) ? dist_fin : dist_now;
    if ((fabs(vx_rel) < 1e-6)) {
        min_dist = fabs(x_rel);
        if ((y_rel > 0 && y_rel_fin < 0) || (y_rel < 0 && y_rel_fin > 0)) {
            minimum_dist = min_dist;
        }
        else {
            minimum_dist = dist_now_fin_min;
        }
    }
    else if (fabs(vy_rel) < 1e-6) {
        min_dist = fabs(y_rel);
        if ((x_rel > 0 && x_rel_fin < 0) || (x_rel < 0 && x_rel_fin > 0)) {
            minimum_dist = min_dist;
        }
        else {
            minimum_dist = dist_now_fin_min;
        }
    }
    else {
        a = -vy_rel / vx_rel;
        b = 1.0;
        c = -a * x_rel - b * y_rel;
        min_dist = pow(c, 2) / (pow(a, 2) + 1.0);
        long double y_per = (-c) / (pow(a, 2) + 1.0);
        if ((y_per > y_rel && y_per < y_rel_fin) || (y_per < y_rel && y_per > y_rel_fin)) {
            minimum_dist = min_dist;
        }
        else {
            minimum_dist = dist_now_fin_min;
        }
    }

    if (minimum_dist < pow((coe * (obj_radius[j] + obj_radius[k])), 2)) {
        return 1;
    }
    else {
        return 0;
    }
}