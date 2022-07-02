#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <iostream>

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

#include "../car_detect/detect_const.h"
#include "../car_detect/coordinate.h"
#include "../car_detect/process_predict.h"
#include "sched_thread.h"

#define __DEBUG__

const int N = 3;        // agent count

sem_t *Fresh, *Mutex_pipe1;

AcrtTime tracker_time;
RVO::Vector2 X[N];    // observed position of agent
RVO::Vector2 V[N];    // observed velocity of agent

struct remote_control {
    int write_fds = -1;
    std::string cmdline;
} rctrl[3];

// receives data from object tracking module
// stores data in global arrays
void *thread_object_tracking(void *args) {
    Coordinate &tracker = *(Coordinate *)args;
    mutex &M = tracker.getMutex();
    const Package &D = tracker.getData();
    Predict pV[3][2], pX[3][2];

    {
        lock_guard<mutex> guard(M);
        for (int i = 0; i < 3; i++) {
            X[i] = RVO::Vector2(D.cars[i].x, D.cars[i].y);
            V[i] = RVO::Vector2(0, 0);
        }
    }
    const long double alpha = 0.5;      // speed inertia
    while (true) {
        const bool use_fake_data = false;
        bool has_new_data = tracker.run(use_fake_data);

        // update velocity
        if (has_new_data) {
            int ms_elapsed = D.time - tracker_time;
            // std::cerr << ms_elapsed << std::endl;
            if (ms_elapsed <= 20) continue;
            sem_wait(Mutex_pipe1);
            for (int i = 0; i < 3; i++) {
                // TODO unit of time
                V[i] = RVO::Vector2(pV[i][0].predict(D.time), pV[i][1].predict(D.time));
                X[i] = RVO::Vector2(pX[i][0].predict(D.time), pX[i][1].predict(D.time));

                RVO::Vector2 newX = RVO::Vector2(D.cars[i].x, D.cars[i].y);
                pX[i][0].push(D.time, newX.x());
                pX[i][1].push(D.time, newX.y());

                // TODO obtain vel directly from the predicted slope of X
                // ASSIGNED TO: yyk
                RVO::Vector2 newV = (newX - X[i]) / ms_elapsed * 1000;
                pV[i][0].push(D.time, newV.x());
                pV[i][1].push(D.time, newV.y());

                // X[i] = alpha * X[i] + (1-alpha) * newX;
                // V[i] = alpha * V[i] + (1-alpha) * newV;
                // X[i] = RVO::Vector2(pX[i][0].predict(tracker_time), pX[i][1].predict(tracker_time));
                // V[i] = RVO::Vector2(pV[i][0].predict(tracker_time), pV[i][1].predict(tracker_time));
            }
            tracker_time = D.time;
            sem_post(Fresh);
            sem_post(Mutex_pipe1);
#ifdef __DEBUG__
            // print velocity in polar and cartesian coordinates
            // printf("v[0] = [%7.2lf, %+7.2lf]: (%.2lf, %.2lf)\n", (double)RVO::abs(V[0]), (double)(V[0].y() / V[0].x()), (double)V[0].x(), (double)V[0].y());
            // printf("v[1] = [%7.2lf, %+7.2lf]: (%.2lf, %.2lf)\n", (double)RVO::abs(V[1]), (double)(V[1].y() / V[1].x()), (double)V[1].x(), (double)V[1].y());
            // printf("v[2] = [%7.2lf, %+7.2lf]: (%.2lf, %.2lf)\n", (double)RVO::abs(V[2]), (double)(V[2].y() / V[2].x()), (double)V[2].x(), (double)V[2].y());

            // print CSV (x, y, vel)
            printf("%+6.2lf, %+6.2lf, %+6.2lf\n", (double)X[0].x(), (double)X[0].y(), (double)RVO::abs(V[0]));

            // print position pairs
            // for (int i = 0; i < 1; i++)
            //     printf("x[%d] = (%.2lf, %.2lf)\n", i, (double)X[i].x(), (double)X[i].y());

            // print time
            // D.time.print();
#endif
        }
    }
    return NULL;
}

void *thread_sched_vel_factor(void *args) {
    ;
    return NULL;
}

void *thread_sched_RVO2(void *args) {
    RVOScheduler sched;

    std::vector<RVO::Vector2> goals;
    goals.push_back(RVO::Vector2(50, 350));
    goals.push_back(RVO::Vector2(390, 340));
    goals.push_back(RVO::Vector2(40, 40));
    sched.setGoal(goals);
    RVO::Vector2 gg(350, 50);

    PackageWithVel sched_data, pilot_data;

    while (true) {
        sem_wait(Fresh);
        sem_wait(Mutex_pipe1);
        sched_data.time = tracker_time;
        for (int i = 0; i < 3; i++) {
            sched_data.cars[i] = X[i];
            sched_data.vels[i] = V[i];
        }
        sem_post(Mutex_pipe1);

        sched.setData(sched_data);
        sched.setGoal(goals);
        sched.step();

        if (1) {
            pilot_data = sched.getNewData();
        } else {
            // pilot_data.vels[0] = RVO::Vector2((float)sched_data.time.msec / 1000, 0.00);
            float periodical = (float)sched_data.time.msec / 1000;
            pilot_data.vels[0] = RVO::Vector2(cos(periodical * 3.1416), sin(periodical * 3.1416));
            pilot_data.time.update();
        }
        // sched.setData(pilot_data);

        char buf[1024];
        float delta_angle, delta_speed;
        for (int i = 0; i < 3; i++) {
            if (rctrl[i].write_fds == -1)
                continue;
            // std::cerr << i << ", " << pilot_data.vels[i] << std::endl;
            RVO::Vector2 v1 = pilot_data.vels[i],
                         v0 = sched_data.vels[i];
            delta_angle =
                asinf(
                    (v1.x() * v0.y() - v1.y() * v0.x())
                    / (RVO::abs(v1) * RVO::abs(v0))
                ) * 2 / 3.1416;
            // printf("delta angle = %+6.4lf\n", (double)delta_angle);
            if (isnan(delta_angle)) delta_angle = 0.00;
            // delta_speed = RVO::abs(v1) / RVO::abs(v0);
            delta_speed = 0.5;
            double dist = RVO::abs(pilot_data.cars[i] - goals[i]);
            if (dist < 70) {
                delta_speed = 0;
                std::swap(goals[i], gg);
                std::cerr << "swap goal!\n";
            }
            int buflen
                = sprintf(buf, "%f %f\n", delta_angle, delta_speed);
            int pos = 0;
            while (pos < buflen)
                pos += write(rctrl[i].write_fds, buf + pos, buflen - pos);
        }
        sched_data = pilot_data;
#ifdef __DEBUG__
        // sched_data.time.print();
        // printf("new v[0] = %lf\n", (double)sched_data.vels[0].x());
        // printf("new v[0] = [%+6.2lf, %+6.2lf]: (%.2lf, %.2lf)\n", (double)RVO::abs(V[0]), (double)(V[0].y() / V[0].x()), (double)V[0].x(), (double)V[0].y());
#endif
    }
    return NULL;
}

// use some <ssh in subprocess> wizardry to control the car.
void *thread_send_msg(void *args) {
    remote_control *pctrl = (remote_control *)args;
    std::string &s = pctrl->cmdline;
    std::vector<std::string> argv;
    size_t pos = 0;
    std::string token;
    while ((pos = s.find(" ")) != std::string::npos) {
        token = s.substr(0, pos);
        argv.push_back(token);
        s = s.substr(pos + std::string(" ").length());
    }
    argv.push_back(s);
    char *arg_ptrs[argv.size() + 1];
    for (int i = 0; i < argv.size(); i++) {
        arg_ptrs[i] = &argv[i][0];
    }
    arg_ptrs[argv.size()] = nullptr;

    int fds[2];
    assert(!pipe(fds));

    pid_t pid = fork();
    if (pid == 0) {
        // child process runs ssh to remote machine
        // setup pipes
        close(STDIN_FILENO);
        assert(dup(fds[0]) == STDIN_FILENO);
        close(fds[0]);
        close(STDOUT_FILENO);
        // close(STDERR_FILENO);
#ifndef __DEBUG__
#endif

        execvp(argv[0].c_str(), arg_ptrs);
    } else {
        // parent is merely a thread; the parent may terminate
        pctrl->write_fds = fds[1];
        close(fds[0]);
    }
    return NULL;
}

int main() {
    int res = 0;
    int total = -1;
    pthread_t pids[10];

    // Caveat: POSIX semaphores not implemented on macOS
    // assert(sem_init(&Fresh, 0, 0) == 0);
    // assert(sem_init(&Mutex_pipe1, 0, 1) == 0);
    Fresh = sem_open("fresh", O_CREAT, 0644, 0);
    Mutex_pipe1 = sem_open("pipe1", O_CREAT, 0644, 1);

    pthread_attr_t attr;
    struct sched_param param = {.sched_priority = 45};

    res = pthread_attr_init(&attr);
    assert(!res);

    if (0)
    {
        res = pthread_attr_setschedpolicy(&attr, SCHED_RR);
        assert(!res);
        res = pthread_attr_setschedparam(&attr, &param);
        assert(!res);
    }

// /*
    // Coordinate tracker(-1);
    Coordinate tracker(1);
    res = pthread_create(&pids[++total], &attr, thread_object_tracking, &tracker);
    assert(!res);
    res = pthread_create(&pids[++total], &attr, thread_sched_RVO2, NULL);
    assert(!res);
// */

    // rctrl[yellow - 1].cmdline = "ssh -t -t pi@192.168.43.196 /home/pi/startup.sh";
    // rctrl[yellow - 1].cmdline = "sleep 10000";
    rctrl[green  - 1].cmdline = "ssh -t -t pi@172.20.10.13";
    rctrl[pink   - 1].cmdline = "ls";

    res = pthread_create(&pids[++total], &attr, thread_send_msg, rctrl+0);
    assert(!res);

/*
    res = pthread_create(&pids[++total], &attr, thread_send_msg, rctrl+1);
    assert(!res);
    // res = pthread_create(&pids[++total], &attr, thread_send_msg, rctrl+2);
    // assert(!res);
*/

    sleep(2);

    res = pthread_attr_destroy(&attr);
    assert(!res);

/*
*/
    // sleep(10);

    for (int i = 0; i <= total; i++) {
        pthread_join(pids[i], NULL);
    }
    return 0;
}