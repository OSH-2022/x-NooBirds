#include <cstdio>
#include <string>
#include <vector>
#include <iostream>

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

#include "../car_detect/coordinate.h"
#include "sched_thread.h"

#define __DEBUG__

const int N = 3;        // agent count

sem_t Fresh, Mutex_pipe1;

AcrtTime tracker_time;
RVO::Vector2 X[N];    // observed position of agent
RVO::Vector2 V[N];    // observed velocity of agent

// receives data from object tracking module
// stores data in global arrays
void *thread_object_tracking(void *args) {
    Coordinate &tracker = *(Coordinate *)args;
    mutex &M = tracker.getMutex();
    const Package &D = tracker.getData();

    {
        lock_guard<mutex> guard(M);
        for (int i = 0; i < 3; i++) {
            X[i] = RVO::Vector2(D.cars[i].x, D.cars[i].y);
            V[i] = RVO::Vector2(0, 0);
        }
    }
    const long double alpha = 0.5;      // speed inertia
    while (true) {
        bool has_new_data = tracker.run(true);

        // update velocity
        if (has_new_data) {
            // TODO use a more sophisticated algorithm to determine speed
            int ms_elapsed = D.time - tracker_time;
            if (ms_elapsed <= 1) continue;
            tracker_time = D.time;
            sem_wait(&Mutex_pipe1);
            for (int i = 0; i < 3; i++) {
                // TODO unit of time
                RVO::Vector2 newX = RVO::Vector2(D.cars[i].x, D.cars[i].y);
                V[i] = alpha * V[i] + (1-alpha) * (newX - X[i]) / ms_elapsed;
                X[i] = newX;
            }
            sem_post(&Fresh);
            sem_post(&Mutex_pipe1);
#ifdef __DEBUG__
            printf("vx[0] = %lf\n", (double)V[0].x());
            D.time.print();
#endif
        }
    }
    return NULL;
}

void *thread_sched_vel_factor(void *args) {
    ;
}

void *thread_sched_RVO2(void *args) {
    PackageWithVel sched_data;
    RVOScheduler sched;
    std::vector<RVO::Vector2> goals;
    goals.push_back(RVO::Vector2(1, 0));
    goals.push_back(RVO::Vector2(2, 2));
    goals.push_back(RVO::Vector2(3, 3));
    sched.setGoal(goals);
    while (true) {
        sem_wait(&Fresh);
        sem_wait(&Mutex_pipe1);
        sched_data.time = tracker_time;
        for (int i = 0; i < 3; i++) {
            sched_data.cars[i] = X[i];
            sched_data.vels[i] = V[i];
        }
        sem_post(&Mutex_pipe1);
        sched.setData(sched_data);
        sched.step();
        sched.setData(sched.getNewData());
    }
    return NULL;
}

// use some <ssh in subprocess> wizardry to control the car.
struct remote_control {
    int write_fds;
    std::string cmdline;
} rctrl[3];

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
    sem_init(&Fresh, 0, 0);
    sem_init(&Mutex_pipe1, 0, 1);

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

    Coordinate tracker(-1);
    res = pthread_create(&pids[++total], &attr, thread_object_tracking, &tracker);
    assert(!res);
    res = pthread_create(&pids[++total], &attr, thread_sched_RVO2, NULL);
    assert(!res);

    rctrl[0].cmdline = "ssh -T wtm@localhost /usr/local/bin/lolcat";
    rctrl[1].cmdline = "ls";
    rctrl[2].cmdline = "ls";

/*
    res = pthread_create(&pids[++total], &attr, thread_send_msg, rctrl+0);
    assert(!res);
    // res = pthread_create(&pids[++total], &attr, thread_send_msg, rctrl+1);
    // assert(!res);
    // res = pthread_create(&pids[++total], &attr, thread_send_msg, rctrl+2);
    // assert(!res);

    sleep(4);

    res = pthread_attr_destroy(&attr);
    assert(!res);

    char buf[] = "echo 1 2 3 4 5 6 7 8\nqwqwqw\n";
    int pos = 0;
    while (pos < sizeof(buf))
        pos += write(rctrl[0].write_fds, buf + pos, sizeof(buf) - pos);
*/

    for (int i = 0; i <= total; i++) {
        pthread_join(pids[i], NULL);
    }
    return 0;
}