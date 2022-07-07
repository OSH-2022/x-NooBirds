#include <cstdio>
#include <algorithm>
#include <mutex>
#include <cmath>
#include "../car_detect/coordinate.h"
#include "../scheduler/schedulerRVO2/src/RVO.h"

const long double pi = 3.14159265358979323846;
const long double adj_step_size = 0.097;
const int adj_times = 10;

#define OBJ_NUM             2
#define PRINT_INTERVAL      2000
#define TIME_BET_SCHE       30
#define SCHE_INTERVAL       900
#define CLOCKWISE           0
#define COUNTER_CLOCKWISE   1

enum sched_method {RVO2, vel_factor};

typedef struct DataPackageWithVel {
    AcrtTime time;
    RVO::Vector2 cars[3], vels[3];
} PackageWithVel;

class RVOScheduler {
public:
    RVOScheduler();
    ~RVOScheduler();

    mutex& getNewDataMutex();
    void setData(const PackageWithVel &);
    void setGoal(const std::vector<RVO::Vector2> &);

    // a monitor to fetch newest data
    const PackageWithVel& getNewData();
    void step();

private:
    std::mutex newDataMutex;
    PackageWithVel data, newData;

    RVO::RVOSimulator *sim;
    std::vector<RVO::Vector2> goals;
};

class HeuristicScheduler {
public:
    HeuristicScheduler();
    ~HeuristicScheduler();

    mutex& getNewDataMutex();
    void setData(const PackageWithVel &);

    const PackageWithVel& getNewData();
    void schedule();

private:
    std::mutex newDataMutex;
    PackageWithVel data, newData;

    int is_adjust[OBJ_NUM] = {0};
    long double initial_vel[OBJ_NUM] = {0.8, 0.8};
    // long double obj_radius[OBJ_NUM] = {150.0, 150.0};
    long double obj_radius[OBJ_NUM] = {300.0, 300.0};
    long double safety_coe = 1.3;
    int slow_down_times[OBJ_NUM] = {0};
    int speed_up_times[OBJ_NUM] = {0};
    int priority[OBJ_NUM] = {0};

    // long double corner_radius = 200.0;

    // long double rect_center[2] = {0.0, 0.0};
    // long double rect_len = 1000.0;

    // long double cir_center[2] = {0.0, 0.0};
    // long double cir_radius = 1200.0;

    // long double tri_center[2];
    // long double tri_len;

    // long double eight_center[2] = {0.0};
    // long double eight_len = 1400.0;
    // long double eight_radius = 1400.0;

    // long double vel_angle[OBJ_NUM]; // TO BE DONE: Return the angles of velocity vetors at a given point and a given direction

    int direction[OBJ_NUM] = {CLOCKWISE};

    enum traj {rect, cir, tri, playground, eight};
    traj obj_traj[OBJ_NUM];
    int obj_traj_seg[OBJ_NUM];

    int corner_case[OBJ_NUM][OBJ_NUM] = {0};
    int slow_car[OBJ_NUM][OBJ_NUM] = {0};

    int unsafe_state_detector(int j, int k, long double jx, long double jy, long double jvx, long double jvy, long double kx, long double ky, long double kvx, long double kvy, long double coe);
};