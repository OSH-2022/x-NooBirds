#include <vector>
#include <iostream>
#include <cmath>

// sim_time unit: ms
// distance unit: mm
// velocity unit: mm / ms

const long double pi = 3.14159265358979323846;

#define OBJ_NUM       3
#define SCHE_INTERVAL 500

#define UPPER_LEFT   0
#define UPPER_EDGE   1
#define UPPER_RIGHT  2
#define RIGHT_EDGE   3
#define BOTTOM_RIGHT 4
#define BOTTOM_EDGE  5
#define BOTTOM_LEFT  6
#define LEFT_EDGE    7

#define CLOCKWISE           0
#define COUNTER_CLOCKWISE   1

long double obj_radius[OBJ_NUM] = {200.0, 200.0, 200.0};
long double obj_x     [OBJ_NUM];
long double obj_y     [OBJ_NUM];
long double obj_vx    [OBJ_NUM];
long double obj_vy    [OBJ_NUM];
long double vel_adjust[OBJ_NUM];
long double safety_coe = 2.5;

long double rect_center[2] = {0.0, 0.0};
long double rect_len = 1000.0;
long double corner_radius = 200.0;
long double cir_center[2] = {0.0, 0.0};
long double cir_radius = 1200.0;
long double tri_center[2];
long double tri_len;

long double angle[OBJ_NUM];

int direction[OBJ_NUM];

int simulation_time = 10;
int sim_time = 0;

enum traj {rect, cir, tri};
traj obj_traj[OBJ_NUM];
int obj_traj_seg[OBJ_NUM];

void scheduler();
int collision_detection();
void print_location();

void main() {
    long double obj_vx_initial[OBJ_NUM] = {0.0};
    long double obj_vy_initial[OBJ_NUM] = {0.0};
    long double obj_x_tmp[OBJ_NUM] = {0.0};
    long double obj_y_tmp[OBJ_NUM] = {0.0};
    long double obj_vx_tmp[OBJ_NUM] = {0.0};
    long double obj_vy_tmp[OBJ_NUM] = {0.0};
    int i = 0;
    int j = 0;
    int k = 0;
    long double dist_sq = 0;

    for (i = 0; i < OBJ_NUM; i++) {
        obj_vx_initial[i] = obj_vx[i];
        obj_vy_initial[i] = obj_vy[i];
        obj_x_tmp[i] = obj_x[i];
        obj_y_tmp[i] = obj_y[i];
        obj_vx_tmp[i] = obj_vx[i];
        obj_vy_tmp[i] = obj_vy[i];
    }

    for (i = 0; i < SCHE_INTERVAL; i++) {
        for (j = 0; j < OBJ_NUM; j++) {
            obj_x_tmp[j] += obj_vx_tmp[j];
            obj_y_tmp[j] += obj_vy_tmp[j];
        }
        for (j = 0; j < OBJ_NUM - 1; j++) {
            for (k = j + 1; k < OBJ_NUM; k++) {
                dist_sq = pow((obj_x_tmp[j] - obj_x_tmp[k]), 2) + pow((obj_y_tmp[j] - obj_y_tmp[k]), 2);
                if (dist_sq < pow((safety_coe * (obj_radius[j] + obj_radius[k])), 2)) {
                    obj_vx_tmp[j] = obj_vx_tmp[j] * ((long double)i / (long double) SCHE_INTERVAL);
                    obj_vy_tmp[j] = obj_vy_tmp[j] * ((long double)i / (long double) SCHE_INTERVAL);
                }
            }
        }
    }

    long double velocity_total_initial = 0.0;
    long double velocity_total_tmp = 0.0;
    for (i = 0; i < OBJ_NUM; i++) {
        velocity_total_initial = sqrt(pow(obj_vx_initial[i], 2) + pow(obj_vy_initial[i], 2));
        velocity_total_tmp = sqrt(pow(obj_vx_tmp[i], 2) + pow(obj_vy_tmp[i], 2));
        vel_adjust[i] = velocity_total_tmp / velocity_total_initial;
    }
}