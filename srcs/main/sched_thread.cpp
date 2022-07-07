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

// void HeuristicScheduler::schedule() {
//     long double x_tmp[OBJ_NUM] = {0.0};
//     long double y_tmp[OBJ_NUM] = {0.0};
//     long double vx_tmp[OBJ_NUM] = {0.0};
//     long double vy_tmp[OBJ_NUM] = {0.0};
//     long double adj_coe = 1.0 - adj_step_size;
//     int adjusted_car = 0;
//     int updated_car = 0;
//     int is_safe = 1;
//     int i = 0;
//     int j = 0;
//     int k = 0;
//     int l = 0;
//     int need_adjust = 0;
//     int need_update = 0;
//     int ret = 0;
//     long double vel_angle[i] = {0.0};

//     for (i = 0; i < OBJ_NUM; i++) {
//         x_tmp[i] = this->data.cars[i].x();
//         y_tmp[i] = this->data.cars[i].y();
//         vx_tmp[i] = this->data.vels[i].x();
//         vy_tmp[i] = this->data.vels[i].y();
//     }

//     for (i = 0; i < OBJ_NUM; i++) {
//         if (vx_tmp[i] > 0) {
//             angle[i] = atan(vy_tmp[i] / vx_tmp[i]);
//         }
//         else if (vy_tmp[i] > 0) {
//             angle[i] = pi + atan(vy_tmp[i] / vx_tmp[i]);
//         }
//     }

//     do {
//         need_adjust = 0;

//         ret = 0;
//         // initial safety detection
//         for (j = 0; j < OBJ_NUM - 1; j++) {
//             for (k = j + 1; k < OBJ_NUM; k++) {
//                 long double coe = safety_coe;
//                 if (corner_case[j][k] == 1) {
//                     ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
//                     if (ret == 0) {
//                         corner_case[j][k] = 0;
//                     }
//                     else {
//                         coe = 1.0;
//                     }
//                 }
//                 ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], coe);
//                 if (ret == 1) {
//                     need_adjust = 1;
//                     break;
//                 }
//             }
//             if (ret == 1) {
//                 break;
//             }
//         }

//         if (ret == 1) {
//             int success = 0;
//             adj_coe = 1.0 - adj_step_size;
//             for (i = 0; i < adj_times; i++) {
//                 ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
//                 if (ret == 0) {
//                     success = 1;
//                     adjusted_car = j;
//                     is_adjust[j] = 1;
//                     break;
//                 }
//                 else {
//                     adj_coe -= adj_step_size;
//                 }
//             }

//             if (success == 0) {
//                 adj_coe = 1.0 - adj_step_size;
//                 for (i = 0; i < adj_times; i++) {
//                     ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], adj_coe * vx_tmp[k], adj_coe * vy_tmp[k], safety_coe);
//                     if (ret == 0) {
//                         success = 1;
//                         adjusted_car = k;
//                         is_adjust[k] = 1;
//                         break;
//                     }
//                     else {
//                         adj_coe -= adj_step_size;
//                     }
//                 }
//             }

//             if (success == 0) {
//                 adj_coe = 0.001;
//                 long double velocity_j = pow(vx_tmp[j], 2) + pow(vy_tmp[j], 2);
//                 long double velocity_k = pow(vx_tmp[k], 2) + pow(vy_tmp[k], 2);
//                 corner_case[j][k] = 1;
//                 long double coe;
//                 coe = 1.0;
//                 ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], adj_coe * vx_tmp[k], adj_coe * vy_tmp[k], coe);
//                 if (ret == 0) {
//                     adjusted_car = k;
//                     is_adjust[k] = 1;
//                     slow_car[j][k] = j;
//                 }
//                 else {
//                     ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], coe);
//                     if (ret == 0) {
//                         adjusted_car = j;
//                         is_adjust[j] = 1;
//                         slow_car[j][k] = k;
//                     }
//                     else {

//                     }
//                 }
//             }
//         }

//         if (need_adjust == 1) {
//             for (i = 0; i < OBJ_NUM; i++) {
//                 if (i == adjusted_car) {
//                     vx_tmp[i] = adj_coe * vx_tmp[i];
//                     vy_tmp[i] = adj_coe * vy_tmp[i];
//                 }
//             }
//         }
//     }
//     while (need_adjust == 1);

//     long double vx_save;
//     long double vy_save;
//     do
//     {
//         need_update = 0;
//         for (l = 0; l < OBJ_NUM; l++) {
//             if (is_adjust[l] == 0) {
//                 continue;
//             }
//             else {
//                 for (i = 0; i < OBJ_NUM; i++) {
//                     if (i == l) {
//                         long double vel_angle;
//                         vel_angle = this->vel_angle[i];
//                         vx_save = vx_tmp[i];
//                         vy_save = vy_tmp[i];
//                         vx_tmp[i] = initial_vel[i] * cos(vel_angle);
//                         vy_tmp[i] = initial_vel[i] * sin(vel_angle);
//                     }
//                 }

//                 ret = 0;
//                 for (j = 0; j < OBJ_NUM - 1; j++) {
//                     for (k = j + 1; k < OBJ_NUM; k++) {
//                         long double coe;
//                         if (corner_case[j][k] == 1 && ((j != l && k != l) || (l == slow_car[j][k]))) {
//                             coe = 1.0;
//                         }
//                         else {
//                             coe = safety_coe * 2.0;
//                         }
//                         ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], coe);
//                         if (ret == 1) {
//                             break;
//                         }
//                     }
//                     if (ret == 1) {
//                         break;
//                     }
//                 }

//                 if (ret == 0) {
//                     need_update = 1;
//                     is_adjust[l] = 0;
//                     break;
//                 }
//                 else {
//                     vx_tmp[l] = vx_save;
//                     vy_tmp[l] = vy_save;
//                 }
//             }
//         }
//     }
//     while (need_update == 1);



//     for (i = 0; i < OBJ_NUM; i++) {
//         this->newData.vels[i] = RVO::Vector2(vx_tmp[i], vy_tmp[i]);
//     }
// }

void HeuristicScheduler::schedule() {
    // *
    // variables declaration
    // *
    // long double vx_initial[OBJ_NUM] = {0.0};
    // long double vy_initial[OBJ_NUM] = {0.0};
    // long double x_initial[OBJ_NUM] = {0.0};
    // long double y_initial[OBJ_NUM] = {0.0};
    long double x_tmp[OBJ_NUM]  = {0.0};
    long double y_tmp[OBJ_NUM]  = {0.0};
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
    int need_adjust_total = 0;
    int ret = 0;
    int cnt = 0;
    int error = 0;

    for (i = 0; i < OBJ_NUM; i++) {
        x_tmp[i] = this->data.cars[i].x();
        y_tmp[i] = this->data.cars[i].y();
        vx_tmp[i] = this->data.vels[i].x();
        vy_tmp[i] = this->data.vels[i].y();
    }
    // *
    // This do-while loop can handle cases in which multiple pairs of cars are in an unsafe state and their speed need to be adjusted for safety.
    // *
    cnt = 0;
    do {
        ret = 0;

        // *
        // Integer variable cnt is used to record the number of iterations do-while loop has performed.
        // If the number of iterations is too high, then the whole system may probably be in a deadlocked state and the error signal should be asserted.
        // *
        cnt++;
        if (cnt > (OBJ_NUM - 1) * OBJ_NUM) {
            error = 1;
            break;
        }

        // *
        // Perform initial safety detection in order to find one and just one pair of cars that are in an unsafe state.
        // If found, the unsafe car pairs are denoted by integer j and k where j < k.
        // *
        for (j = 0; j < OBJ_NUM - 1; j++) {
            for (k = j + 1; k < OBJ_NUM; k++) {
                long double coe;
                if (corner_case[j][k] == 1) {
                    coe = 1.0;
                }
                else {
                    coe = safety_coe;
                }
                ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], coe);
                if (ret == 1) {
                    need_adjust = 1;
                    need_adjust_total = 1;
                    break;
                }
            }
            if (ret == 1) {
                break;
            }
        }

        // *
        // If need_adjust = 0, then all pairs of cars are in a safe state so that the scheduler should jump out of the do-while loop.
        // *
        if (ret == 0) {
            need_adjust = 0;
        }

        // if (sim_time > 41370 && obj_vx[0] > 0) {
        //     Sleep(1);
        // }
        // if (obj_traj_seg[0] == EIGHT_LEFT_CIR && angle[i] < pi && obj_vx[0] > 0) {
        //     Sleep(1);
        // }
        // if (j == 0 && k == 1) {
        //     Sleep(1);
        // // }
        // if (j == 0 && k == 2 && sim_time > 43170) {
        //     Sleep(1);
        // }
        // if (j == 1 && k == 2) {
        //     Sleep(1);
        // }

        // *
        // Attempt to handle the unsafe state by first increasing the speed of one car.
        // Integer success is used to indicate whether we have solved the unsafe problem.
        // Increase the speed of one car without exceeding the maximum speed of car can lead to better traffic performances.
        // *
        long double amplify_factor = 2.0;
        if (ret == 1) {
            int success = 0;

            long double vel_total_j;
            long double vel_total_k;
            vel_total_j = sqrt(pow(vx_tmp[j], 2) + pow(vy_tmp[j], 2));
            vel_total_k = sqrt(pow(vx_tmp[k], 2) + pow(vy_tmp[k], 2));

            // *
            // If car j has a higher priority than car k, the algorithm should consider to increase the speed of car j before car k or decrease the speed of car j after car k in order to reduce the traffic time of car j.
            // *
            if (priority[j] > priority[k]) {
                while (amplify_factor * vel_total_j < initial_vel[j]) {
                    ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], amplify_factor * vx_tmp[j], amplify_factor * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
                    if (ret == 0) {
                        success = 1;
                        adjusted_car = j;
                        speed_up_times[j]++;
                        is_adjust[j] = 1;
                        vx_tmp[j] *= amplify_factor;
                        vy_tmp[j] *= amplify_factor;
                        break;
                    }
                    amplify_factor *= 2.0;
                }

                if (success == 0) {
                    amplify_factor = 2.0;
                    while (amplify_factor * vel_total_k < initial_vel[k]) {
                        ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], amplify_factor * vx_tmp[k], amplify_factor * vy_tmp[k], safety_coe);
                        if (ret == 0) {
                            success = 1;
                            adjusted_car = k;
                            speed_up_times[k]++;
                            is_adjust[k] = 1;
                            vx_tmp[k] *= amplify_factor;
                            vy_tmp[k] *= amplify_factor;
                            break;
                        }
                        amplify_factor *= 2.0;
                    }
                }


                // *
                // If the attempts to increase the speed of one car fails, try next to decrease the speed of the car.
                // *
                if (success == 0) {
                    adj_coe = 1.0 - adj_step_size;
                    for (i = 0; i < adj_times; i++) {
                        ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], adj_coe * vx_tmp[k], adj_coe * vy_tmp[k], safety_coe);
                        if (ret == 0) {
                            success = 1;
                            adjusted_car = k;
                            slow_down_times[k]++;
                            is_adjust[k] = 1;
                            break;
                        }
                        else {
                            adj_coe -= adj_step_size;
                        }
                    }
                }

                if (success == 0) {
                    adj_coe = 1.0 - adj_step_size;
                    for (i = 0; i < adj_times; i++) {
                        ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
                        if (ret == 0) {
                            success = 1;
                            adjusted_car = j;
                            slow_down_times[j]++;
                            is_adjust[j] = 1;
                            break;
                        }
                        else {
                            adj_coe -= adj_step_size;
                        }
                    }
                }
            }
            // *
            // If two cars have the same priority, then fairness rule comes into effect.
            // *
            else if (priority[j] == priority[k]) {
                if (speed_up_times[j] >= speed_up_times[k]) {
                    while (amplify_factor * vel_total_k < initial_vel[k]) {
                        ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], amplify_factor * vx_tmp[k], amplify_factor * vy_tmp[k], safety_coe);
                        if (ret == 0) {
                            success = 1;
                            adjusted_car = k;
                            speed_up_times[k]++;
                            is_adjust[k] = 1;
                            vx_tmp[k] *= amplify_factor;
                            vy_tmp[k] *= amplify_factor;
                            break;
                        }
                        amplify_factor *= 2.0;
                    }

                    if (success == 0) {
                        amplify_factor = 2.0;
                        while (amplify_factor * vel_total_j < initial_vel[j]) {
                            ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], amplify_factor * vx_tmp[j], amplify_factor * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
                            if (ret == 0) {
                                success = 1;
                                adjusted_car = j;
                                speed_up_times[j]++;
                                is_adjust[j] = 1;
                                vx_tmp[j] *= amplify_factor;
                                vy_tmp[j] *= amplify_factor;
                                break;
                            }
                            amplify_factor *= 2.0;
                        }
                    }
                }
                else {
                    while (amplify_factor * vel_total_j < initial_vel[j]) {
                        ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], amplify_factor * vx_tmp[j], amplify_factor * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
                        if (ret == 0) {
                            success = 1;
                            adjusted_car = j;
                            speed_up_times[j]++;
                            is_adjust[j] = 1;
                            vx_tmp[j] *= amplify_factor;
                            vy_tmp[j] *= amplify_factor;
                            break;
                        }
                        amplify_factor *= 2.0;
                    }

                    if (success == 0) {
                        amplify_factor = 2.0;
                        while (amplify_factor * vel_total_k < initial_vel[k]) {
                            ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], amplify_factor * vx_tmp[k], amplify_factor * vy_tmp[k], safety_coe);
                            if (ret == 0) {
                                success = 1;
                                adjusted_car = k;
                                speed_up_times[k]++;
                                is_adjust[k] = 1;
                                vx_tmp[k] *= amplify_factor;
                                vy_tmp[k] *= amplify_factor;
                                break;
                            }
                            amplify_factor *= 2.0;
                        }
                    }
                }

                if (success == 0) {
                    if (slow_down_times[j] > slow_down_times[k]) {
                        adj_coe = 1.0 - adj_step_size;
                        for (i = 0; i < adj_times; i++) {
                            ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], adj_coe * vx_tmp[k], adj_coe * vy_tmp[k], safety_coe);
                            if (ret == 0) {
                                success = 1;
                                adjusted_car = k;
                                slow_down_times[k]++;
                                is_adjust[k] = 1;
                                break;
                            }
                            else {
                                adj_coe -= adj_step_size;
                            }
                        }

                        if (success == 0) {
                            adj_coe = 1.0 - adj_step_size;
                            for (i = 0; i < adj_times; i++) {
                                ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
                                if (ret == 0) {
                                    success = 1;
                                    adjusted_car = j;
                                    slow_down_times[j]++;
                                    is_adjust[j] = 1;
                                    break;
                                }
                                else {
                                    adj_coe -= adj_step_size;
                                }
                            }
                        }
                    }
                    else {
                        adj_coe = 1.0 - adj_step_size;
                        for (i = 0; i < adj_times; i++) {
                            ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
                            if (ret == 0) {
                                success = 1;
                                adjusted_car = j;
                                slow_down_times[j]++;
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
                                    slow_down_times[k]++;
                                    is_adjust[k] = 1;
                                    break;
                                }
                                else {
                                    adj_coe -= adj_step_size;
                                }
                            }
                        }
                    }
                }
            }
            else {
                while (amplify_factor * vel_total_k < initial_vel[k]) {
                    ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], amplify_factor * vx_tmp[k], amplify_factor * vy_tmp[k], safety_coe);
                    if (ret == 0) {
                        success = 1;
                        adjusted_car = k;
                        speed_up_times[k]++;
                        is_adjust[k] = 1;
                        vx_tmp[k] *= amplify_factor;
                        vy_tmp[k] *= amplify_factor;
                        break;
                    }
                    amplify_factor *= 2.0;
                }

                if (success == 0) {
                    amplify_factor = 2.0;
                    while (amplify_factor * vel_total_j < initial_vel[j]) {
                        ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], amplify_factor * vx_tmp[j], amplify_factor * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
                        if (ret == 0) {
                            success = 1;
                            adjusted_car = j;
                            speed_up_times[j]++;
                            is_adjust[j] = 1;
                            vx_tmp[j] *= amplify_factor;
                            vy_tmp[j] *= amplify_factor;
                            break;
                        }
                        amplify_factor *= 2.0;
                    }
                }

                // *
                // If the attempts to increase the speed of one car fails, try next to decrease the speed of the car.
                // *
                if (success == 0) {
                    adj_coe = 1.0 - adj_step_size;
                    for (i = 0; i < adj_times; i++) {
                        ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
                        if (ret == 0) {
                            success = 1;
                            adjusted_car = j;
                            slow_down_times[j]++;
                            is_adjust[j] = 1;
                            break;
                        }
                        else {
                            adj_coe -= adj_step_size;
                        }
                    }
                }

                if (success == 0) {
                    adj_coe = 1.0 - adj_step_size;
                    for (i = 0; i < adj_times; i++) {
                        ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], adj_coe * vx_tmp[k], adj_coe * vy_tmp[k], safety_coe);
                        if (ret == 0) {
                            success = 1;
                            adjusted_car = k;
                            slow_down_times[k]++;
                            is_adjust[k] = 1;
                            break;
                        }
                        else {
                            adj_coe -= adj_step_size;
                        }
                    }
                }
            }

            // *
            // If the above methods all fail, then the two cars are in a corner state (the corner/critical state is entered because the scheduling algorithm does not know a single route of the car and is not always running(there is a given interval between two scheduling processes)).
            // Try to stop one car to avoid accident.
            // *
            if (success == 0) {
                adj_coe = 0.001;
                long double velocity_j = pow(obj_vx[j], 2) + pow(obj_vy[j], 2);
                long double velocity_k = pow(obj_vx[k], 2) + pow(obj_vy[k], 2);
                corner_case[j][k] = 1;
                long double coe;
                coe = 1.0;
                ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], adj_coe * vx_tmp[k], adj_coe * vy_tmp[k], coe);
                if (ret == 0) {
                    adjusted_car = k;
                    slow_down_times[k]++;
                    is_adjust[k] = 1;
                    slow_car[j][k] = j;
                }
                else {
                    ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], coe);
                    if (ret == 0) {
                        adjusted_car = j;
                        slow_down_times[j]++;
                        is_adjust[j] = 1;
                        slow_car[j][k] = k;
                    }
                    else { 
                    }
                }

                // // *
                // // If the above initial methods fail, use this heuristic method to handle this corner case.
                // // This method slows down the faster car.
                // // *
                // int cond1 = (x_tmp[j] < x_tmp[k]) && (vx_tmp[k] - vx_tmp[j] > 0);
                // int cond2 = (x_tmp[j] > x_tmp[k]) && (vx_tmp[k] - vx_tmp[k] < 0);
                // int cond3 = (fabs(y_tmp[j] - y_tmp[k]) < 20.0) && (fabs(vy_tmp[j] - vy_tmp[k] < 20.0));
                // if (cond3 && (cond2 || cond1)) {
                //     if (fabs(vx_tmp[j]) < fabs(vx_tmp[k])) {
                //         adjusted_car = j;
                //         is_adjust[j] = 1;
                //         slow_car[j][k] = k;
                //     }
                //     else {
                //         adjusted_car = k;
                //         is_adjust[k] = 1;
                //         slow_car[j][k] = j;
                //     }
                // }
                // else {
                //     if (velocity_j > velocity_k) {
                //         // ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], 1.0);
                //         // if (ret == 1) {
                //         //     need_adjust = 0;
                //         // }
                //         // else {
                //             adjusted_car = j;
                //             is_adjust[j] = 1;
                //             slow_car[j][k] = k;
                //         // }
                //     }
                //     else {
                //         // ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], 1.0);
                //         // if (ret == 1) {
                //         //     need_adjust = 0;
                //         // }
                //         // else {
                //             adjusted_car = k;
                //             is_adjust[k] = 1;
                //             slow_car[j][k] = j;
                //     }
                // }

                // *
                // If the above initial methods fail, use this heuristic method to handle this corner case.
                // This method tries to slow down the chasing car. If finds the chasing car by dot product.
                // *
                // long double r_jk[2] = {x_tmp[j] - x_tmp[k], y_tmp[j] - y_tmp[k]};
                // long double product = vx_tmp[k] * r_jk[0] + vy_tmp[k] * r_jk[1];
                // if (product > 0) {
                //     adjusted_car = k;
                //     is_adjust[k] = 1;
                //     slow_car[j][k] = j;
                // }
                // else {
                //     adjusted_car = j;
                //     is_adjust[j] = 1;
                //     slow_car[j][k] = k;
                // }
            }
        }
        // // *
        // // only handles one case
        // // *
        // else { // try to update
        //     for (l = 0; l < OBJ_NUM; l++) {
        //         if (is_adjust[l] == 0) {
        //             continue;
        //         }
        //         else {
        //             for (i = 0; i < OBJ_NUM; i++) {
        //                 if (i == l) {
        //                     x_tmp[i] = obj_x[i];
        //                     y_tmp[i] = obj_y[i];
        //                     long double vel_angle;
        //                     vel_angle = return_velocity_angle(i);
        //                     vx_tmp[i] = initial_vel[i] * cos(vel_angle);
        //                     vy_tmp[i] = initial_vel[i] * sin(vel_angle);
        //                 }
        //                 else {
        //                     x_tmp[i] = obj_x[i];
        //                     y_tmp[i] = obj_y[i];
        //                     vx_tmp[i] = obj_vx[i];
        //                     vy_tmp[i] = obj_vy[i];
        //                 }
        //             }

        //             ret = 0;
        //             for (j = 0; j < OBJ_NUM - 1; j++) {
        //                 for (k = j + 1; k < OBJ_NUM; k++) {
        //                     long double coe;
        //                     if (l == slow_car[j][k] && corner_case[j][k] == 1) {
        //                         coe = 1.0;
        //                     }
        //                     else {
        //                         coe = safety_coe * 2.0;
        //                     }
        //                     ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], coe);
        //                     if (ret == 1) {
        //                         break;
        //                     }
        //                 }
        //                 if (ret == 1) {
        //                     break;
        //                 }
        //             }
        //             if (ret == 0) {
        //                 need_update = 1;
        //                 is_adjust[l] = 0;
        //                 updated_car = l;
        //                 break;
        //             }
        //         }
        //     }
        //     if (ret == 1) {
        //         need_update = 0;
        //     }
        // }

        // *
        // Temporarily adjust the velocity of the adjusted car by changing its v{x,y}_tmp[].
        // *
        vx_tmp[adjusted_car] = adj_coe * vx_tmp[adjusted_car];
        vy_tmp[adjusted_car] = adj_coe * vy_tmp[adjusted_car];
        // // *
        // // only handles one case
        // // *
        // if (need_adjust == 0) {
        //     if (need_update == 1) {
        //         int flag = 1;
        //         for (k = updated_car + 1; k < OBJ_NUM; k++) {
        //             if (corner_case[updated_car][k] == 1) {
        //                 corner_case[updated_car][k] = 0;
        //                 flag = 0;
        //                 break;
        //             }
        //         }
        //         if (flag == 1) {
        //             for (j = 0; j < updated_car; j++) {
        //                 if (corner_case[j][updated_car] == 1) {
        //                     corner_case[j][updated_car] = 0;
        //                     break;
        //                 }
        //             }
        //         }
        //         for (i = 0; i < OBJ_NUM; i++) {
        //             if (i == updated_car) {
        //                 long double vel_angle;
        //                 vel_angle = return_velocity_angle(i);
        //                 vel_adjust[i][0] = initial_vel[i] * cos(vel_angle);
        //                 vel_adjust[i][1] = initial_vel[i] * sin(vel_angle);
        //                 vx_tmp[i] = vel_adjust[i][0];
        //                 vy_tmp[i] = vel_adjust[i][1];
        //             }
        //             else {
        //                 // vel_adjust[i][0] = obj_vx[i];
        //                 // vel_adjust[i][1] = obj_vy[i];
        //             }
        //         }
        //     }
        //     else {
        //         for (i = 0; i < OBJ_NUM; i++) {
        //             // vel_adjust[i][0] = obj_vx[i];
        //             // vel_adjust[i][1] = obj_vy[i];
        //         }
        //     }
        // }
        // else {
        //     for (i = 0; i < OBJ_NUM; i++) {
        //         if (i == adjusted_car) {
        //             // vel_adjust[i][0] = adj_coe * obj_vx[i];
        //             // vel_adjust[i][1] = adj_coe * obj_vy[i];
        //             vx_tmp[i] = vel_adjust[i][0];
        //             vy_tmp[i] = vel_adjust[i][1];
        //         }
        //         else {
        //             // vel_adjust[i][0] = obj_vx[i];
        //             // vel_adjust[i][1] = obj_vy[i];
        //         }
        //     }
        // }
    }
    while (need_adjust == 1);


    // *
    // Recover the adjusted car's velocity to its initial maximum velocity so that the car that has been slowed down could gain speed again.
    // No iterations here.
    // *
    long double vx_save;
    long double vy_save;
    for (l = 0; l < OBJ_NUM; l++) {
        if (is_adjust[l] == 0) {
            continue;
        }
        else {
            for (i = 0; i < OBJ_NUM; i++) {
                if (i == l) {
                    // x_tmp[i] = obj_x[i];
                    // y_tmp[i] = obj_y[i];
                    long double vel_angle;
                    vel_angle = return_velocity_angle(i);
                    vx_save = vx_tmp[i];
                    vy_save = vy_tmp[i];
                    vx_tmp[i] = initial_vel[i] * cos(vel_angle);
                    vy_tmp[i] = initial_vel[i] * sin(vel_angle);
                }
                else {
                    // x_tmp[i] = obj_x[i];
                    // y_tmp[i] = obj_y[i];
                    // vx_tmp[i] = obj_vx[i];
                    // vy_tmp[i] = obj_vy[i];
                }
            }

            ret = 0;
            for (j = 0; j < OBJ_NUM - 1; j++) {
                for (k = j + 1; k < OBJ_NUM; k++) {
                    long double coe;
                    if (l == slow_car[j][k] && corner_case[j][k] == 1) {
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
                // need_update = 1;
                is_adjust[l] = 0;
                updated_car = l;
                break;
            }
            else {
                vx_tmp[l] = vx_save;
                vy_tmp[l] = vy_save;
            }
        }
    }

    // *
    // find the deadlocked state of the system and recover from the detected deadlock
    // If two cars are running at very slow speed, then the deadlock may occur. Try to speed up the car.
    // *
    long double vel_total[OBJ_NUM] = {0};
    for (i = 0; i < OBJ_NUM; i++) {
        vel_total[i] = sqrt(pow(vx_tmp[i], 2) + pow(vy_tmp[i], 2));
    }
    long double angle_j;
    long double angle_k;
    for (j = 0; j < OBJ_NUM - 1; j++) {
        for (k = j + 1; k < OBJ_NUM; k++) {
            if (fabs(vel_total[j]) < 0.01 && fabs(vel_total[k] < 0.01)) {
                angle_j = return_velocity_angle(j);
                angle_k = return_velocity_angle(k);
                ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], 0.0, 0.0, x_tmp[k], y_tmp[k], initial_vel[k] * cos(angle_k), initial_vel[k] * sin(angle_k), safety_coe - 1.0);
                if (ret == 0) {
                    vx_tmp[j] = 0.0;
                    vy_tmp[j] = 0.0;
                    vx_tmp[k] = initial_vel[k] * cos(angle_k);
                    vy_tmp[k] = initial_vel[k] * sin(angle_k);
                }
                else {
                    ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], initial_vel[j] * cos(angle_j), initial_vel[j] * sin(angle_j), x_tmp[k], y_tmp[k], 0.0, 0.0, safety_coe - 1.0);
                    if (ret == 0) {
                        vx_tmp[j] = initial_vel[j] * cos(angle_j);
                        vy_tmp[j] = initial_vel[j] * sin(angle_j);
                        vx_tmp[k] = 0.0;
                        vy_tmp[k] = 0.0;
                    }
                    else {
                    }
                }
            }
        }
    }

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