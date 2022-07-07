#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <Windows.h>

// sim_time unit: ms
// distance unit: mm
// velocity unit: mm / ms

const long double pi = 3.14159265358979323846;
const long double adj_step_size = 0.097;
const int adj_times = 10;

#define OBJ_NUM             3

#define PRINT_INTERVAL      400
#define TIME_BET_SCHE       30
#define SCHE_INTERVAL       1400

int is_adjust[OBJ_NUM] = {0};
long double initial_vel[OBJ_NUM];

// *
// used for route rendering: rect, tri, eight
// *
#define UPPER_LEFT   0
#define UPPER_EDGE   1
#define UPPER_RIGHT  2
#define RIGHT_EDGE   3
#define BOTTOM_RIGHT 4
#define BOTTOM_EDGE  5
#define BOTTOM_LEFT  6
#define LEFT_EDGE    7

#define TRI_UP_CORNER    0
#define TRI_RIGHT_EDGE   1
#define TRI_RIGHT_CORNER 2
#define TRI_DOWN_EDGE    3
#define TRI_LEFT_CORNER  4
#define TRI_LEFT_EDGE    5

#define EIGHT_VER_LINE   0
#define EIGHT_RIGHT_CIR  1
#define EIGHT_PAR_LINE   2
#define EIGHT_LEFT_CIR   3

// used to set the moving direction of cars
#define CLOCKWISE           0
#define COUNTER_CLOCKWISE   1

// *
// Global variables used by the scheduler
// used to implement priority-based scheduling and fair scheduling rule
int slow_down_times[OBJ_NUM] = {0};
int speed_up_times[OBJ_NUM] = {0};
// The bigger the number is, the higher priority the car has.
int priority[OBJ_NUM] = {0};
int corner_case[OBJ_NUM][OBJ_NUM] = {0};
int slow_car[OBJ_NUM][OBJ_NUM] = {0};
// *


// collision radius for each car
long double obj_radius[OBJ_NUM] = {200.0, 200.0, 200.0};

// real data for kinetic information for each car
long double obj_x     [OBJ_NUM];
long double obj_y     [OBJ_NUM];
long double obj_vx    [OBJ_NUM];
long double obj_vy    [OBJ_NUM];

// output by the scheduler: recommended speed of each car
long double vel_adjust[OBJ_NUM][2];


long double safety_coe = 1.5;


// parameters for route rendering and simulation
long double corner_radius = 200.0;

long double rect_center[2] = {0.0, 0.0};
long double rect_len = 1000.0;

long double cir_center[2] = {0.0, 0.0};
long double cir_radius = 1200.0;

long double tri_center[2];
long double tri_len;

long double eight_center[2] = {0.0};
long double eight_len = 1400.0;
long double eight_radius = 1400.0;

long double angle[OBJ_NUM];

int direction[OBJ_NUM];

int simulation_time = 1000000000;
int sim_time = 0;

enum traj {rect, cir, tri, playground, eight};
traj obj_traj[OBJ_NUM];
int obj_traj_seg[OBJ_NUM];

// *
// prototype declarations for functions
// *
void initialize_car(int i, traj traj, int traj_seg, long double x, long double y, long double angle, long double vel, int direction);
void scheduler_1();
void scheduler_2();
int collision_detection();
void print_location_verbose();
void print_location_concise();
long double return_velocity_angle(int id);
int unsafe_state_detector(int j, int k, long double jx, long double jy, long double jvx, long double jvy, long double kx, long double ky, long double kvx, long double kvy, long double coe);

// *
// Simulator
// *
int main(int argc, char **argv) {
    obj_x[0] = eight_center[0];
    obj_y[0] = eight_center[1] - eight_len;
    initial_vel[0] = 0.3;
    obj_vx[0] = 0.0;
    obj_vy[0] = 0.3;
    obj_traj[0] = eight;
    obj_traj_seg[0] = EIGHT_VER_LINE;
    direction[0] = CLOCKWISE;
    angle[0] = pi / 2;

    obj_x[1] = eight_center[0] + eight_len;
    obj_y[1] = eight_center[1];
    initial_vel[1] = 0.4;
    obj_vx[1] = -0.4;
    obj_vy[1] = 0.0;
    obj_traj[1] = eight;
    obj_traj_seg[1] = EIGHT_PAR_LINE;
    direction[1] = CLOCKWISE;
    angle[1] = pi;

    obj_x[2] = eight_center[0] + eight_len;
    obj_y[2] = eight_center[1];
    initial_vel[2] = 0.6;
    obj_vx[2] = -0.6;
    obj_vy[2] = 0.0;
    obj_traj[2] = eight;
    obj_traj_seg[2] = EIGHT_LEFT_CIR;
    direction[2] = CLOCKWISE;
    angle[2] = pi;

    long double corner_left    = 0.0;
    long double move_dist      = 0.0;
    long double angle_adjust   = 0.0;
    long double velocity_total = 0.0;
    long double deviation      = 0.0;

    for (sim_time = 0; sim_time < simulation_time; sim_time++) {
        for (int i = 0; i < OBJ_NUM; i++) {
            switch (obj_traj[i]) {
            case rect:
                switch (obj_traj_seg[i]) {
                case UPPER_LEFT:
                    if (direction[i] == CLOCKWISE) {
                        corner_left = corner_radius * (angle[i] - pi / 2);
                        move_dist = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        velocity_total = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        if (move_dist > corner_left) {
                            obj_traj_seg[i] = UPPER_EDGE;
                            obj_x[i] = rect_center[0] - rect_len + move_dist - corner_left;
                            obj_y[i] = rect_center[1] + rect_len + corner_radius;
                            obj_vx[i] = velocity_total;
                            obj_vy[i] = 0.0;
                        }
                        else {
                            angle[i] = angle[i] - (move_dist) / corner_radius;
                            angle_adjust = angle[i] - pi / 2;
                            obj_x[i] = rect_center[0] - rect_len - corner_radius * sin(angle_adjust);
                            obj_y[i] = rect_center[1] + rect_len + corner_radius * cos(angle_adjust);
                            obj_vx[i] = velocity_total * cos(angle_adjust);
                            obj_vy[i] = velocity_total * sin(angle_adjust);
                        }
                    }
                    else {
                    }
                    break;
                case UPPER_EDGE:
                    if (direction[i] == CLOCKWISE) {
                        deviation = obj_vx[i] - (rect_center[0] + rect_len - obj_x[i]);
                        if (deviation > 0) {
                            obj_traj_seg[i] = UPPER_RIGHT;
                            angle[i] = pi / 2 - deviation / corner_radius;
                            obj_x[i] = rect_center[0] + rect_len + corner_radius * cos(angle[i]);
                            obj_y[i] = rect_center[1] + rect_len + corner_radius * sin(angle[i]);
                            obj_vx[i] = obj_vx[i] * sin(angle[i]);
                            obj_vy[i] = -obj_vx[i] * cos(angle[i]);
                        }
                        else {
                            obj_x[i] = obj_x[i] + obj_vx[i];
                        }
                    }
                    else {
                    }
                    break;
                case UPPER_RIGHT:
                    if (direction[i] == CLOCKWISE) {
                        corner_left = corner_radius * angle[i];
                        move_dist = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        velocity_total = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        deviation = move_dist - corner_left;
                        if (deviation > 0) {
                            obj_traj_seg[i] = RIGHT_EDGE;
                            obj_x[i] = rect_center[0] + rect_len + corner_radius;
                            obj_y[i] = rect_center[1] + rect_len - deviation;
                            obj_vx[i] = 0.0;
                            obj_vy[i] = -velocity_total;
                        }
                        else {
                            angle[i] = angle[i] - (move_dist) / corner_radius;
                            obj_x[i] = rect_center[0] + rect_len + corner_radius * cos(angle[i]);
                            obj_y[i] = rect_center[1] + rect_len + corner_radius * sin(angle[i]);
                            obj_vx[i] = velocity_total * sin(angle[i]);
                            obj_vy[i] = -velocity_total * cos(angle[i]);
                        }
                    }
                    else {
                    }
                    break;
                case RIGHT_EDGE:
                    if (direction[i] == CLOCKWISE) {
                        deviation = -obj_vy[i] - obj_y[i] + rect_center[1] - rect_len;
                        if (deviation > 0) {
                            obj_traj_seg[i] = BOTTOM_RIGHT;
                            angle[i] = 2 * pi - deviation / corner_radius;
                            angle_adjust = angle[i] - 3 * pi / 2;
                            obj_x[i] = rect_center[0] + rect_len + corner_radius * sin(angle_adjust);
                            obj_y[i] = rect_center[1] - rect_len - corner_radius * cos(angle_adjust);
                            obj_vx[i] = obj_vy[i] * cos(angle_adjust);
                            obj_vy[i] = obj_vy[i] * sin(angle_adjust);
                        }
                        else {
                            obj_y[i] = obj_y[i] + obj_vy[i];
                        }
                    }
                    else {
                    }
                    break;
                case BOTTOM_RIGHT:
                    if (direction[i] == CLOCKWISE) {
                        corner_left = corner_radius * (angle[i] - 3 * pi / 2);
                        move_dist = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        velocity_total = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        deviation = move_dist - corner_left;
                        if (deviation > 0) {
                            obj_traj_seg[i] = BOTTOM_EDGE;
                            obj_x[i] = rect_center[0] + rect_len - deviation;
                            obj_y[i] = rect_center[1] - rect_len - corner_radius;
                            obj_vx[i] = -velocity_total;
                            obj_vy[i] = 0.0;
                        }
                        else {
                            angle[i] = angle[i] - move_dist / corner_radius;
                            angle_adjust = angle[i] - 3 * pi / 2;
                            obj_x[i] = rect_center[0] + rect_len + corner_radius * sin(angle_adjust);
                            obj_y[i] = rect_center[1] - rect_len - corner_radius * cos(angle_adjust);
                            obj_vx[i] = -velocity_total * cos(angle_adjust);
                            obj_vy[i] = -velocity_total * sin(angle_adjust);
                        }
                    }
                    else {
                    }
                    break;
                case BOTTOM_EDGE:
                    if (direction[i] == CLOCKWISE) {
                        deviation = -obj_vx[i] - obj_x[i] + rect_center[0] - rect_len;
                        if (deviation > 0) {
                            obj_traj_seg[i] = BOTTOM_LEFT;
                            angle[i] = 3 * pi / 2 - deviation / corner_radius;
                            angle_adjust = 3 * pi / 2 - angle[i];
                            obj_x[i] = rect_center[0] - rect_len - corner_radius * sin(angle_adjust);
                            obj_y[i] = rect_center[1] - rect_len - corner_radius * cos(angle_adjust);
                            obj_vx[i] = obj_vx[i] * cos(angle_adjust);
                            obj_vy[i] = -obj_vx[i] * sin(angle_adjust);
                        }
                        else {
                            obj_x[i] = obj_x[i] + obj_vx[i];
                        }
                    }
                    else {
                    }
                    break;
                case BOTTOM_LEFT:
                    if (direction[i] == CLOCKWISE) {
                        corner_left = corner_radius * (angle[i] - pi);
                        move_dist = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        velocity_total = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        deviation = move_dist - corner_left;
                        if (deviation > 0) {
                            obj_traj_seg[i] = LEFT_EDGE;
                            obj_x[i] = rect_center[0] - rect_len - corner_radius;
                            obj_y[i] = rect_center[1] - rect_len + deviation;
                            obj_vx[i] = 0.0;
                            obj_vy[i] = velocity_total;
                        }
                        else {
                            angle[i] = angle[i] - move_dist / corner_radius;
                            angle_adjust = 3 * pi / 2 - angle[i];
                            obj_x[i] = rect_center[0] - rect_len - corner_radius * sin(angle_adjust);
                            obj_y[i] = rect_center[1] - rect_len - corner_radius * cos(angle_adjust);
                            obj_vx[i] = -velocity_total * cos(angle_adjust);
                            obj_vy[i] = velocity_total * sin(angle_adjust);
                        }
                    }
                    else {
                    }
                    break;
                case LEFT_EDGE:
                    if (direction[i] == CLOCKWISE) {
                        deviation = obj_vy[i] - rect_center[1] - rect_len + obj_y[i];
                        if (deviation > 0) {
                            obj_traj_seg[i] = UPPER_LEFT;
                            angle[i] = pi - deviation / corner_radius;
                            angle_adjust = angle[i] - pi / 2;
                            obj_x[i] = rect_center[0] - rect_len - corner_radius * sin(angle_adjust);
                            obj_y[i] = rect_center[1] + rect_len + corner_radius * cos(angle_adjust);
                            obj_vx[i] = velocity_total * cos(angle_adjust);
                            obj_vy[i] = velocity_total * sin(angle_adjust);
                        }
                        else {
                            obj_y[i] = obj_y[i] + obj_vy[i];
                        }
                    }
                    else {
                    }
                    break;
                default:
                    break;
                }
                break;
            case cir:
                if (direction[i] == CLOCKWISE) {
                    velocity_total = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                    angle[i] = angle[i] - velocity_total / cir_radius;
                    if (angle[i] < 0) {
                        angle[i] = angle[i] = 2 * pi;
                    }
                    obj_x[i] = cir_center[0] + cir_radius * cos(angle[i]);
                    obj_y[i] = cir_center[1] + cir_radius * sin(angle[i]);
                    obj_vx[i] = velocity_total * sin(angle[i]);
                    obj_vy[i] = -velocity_total * cos(angle[i]);
                }
                else {
                }
                break;
            case tri:
                switch (obj_traj_seg[i]) {
                case TRI_UP_CORNER:
                    if (direction[i] == CLOCKWISE) {
                        corner_left = corner_radius * (angle[i] - pi / 6);
                        velocity_total = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        deviation = velocity_total - corner_left;
                        if (deviation > 0) {
                            obj_traj_seg[i] = TRI_RIGHT_EDGE;
                            obj_x[i] = cir_center[0] + sin(pi / 3) * corner_radius + deviation / 2;
                            obj_y[i] = cir_center[1] + tri_len + corner_radius / 2 - deviation * sin(pi / 3);
                            obj_vx[i] = velocity_total / 2;
                            obj_vy[i] = -velocity_total * sin(pi / 3);
                        }
                        else {
                            angle[i] = angle[i] - velocity_total / corner_radius;
                            obj_x[i] = tri_center[0] + corner_radius * cos(angle[i]);
                            obj_y[i] = tri_center[1] + tri_len + corner_radius * sin(angle[i]);
                            obj_vx[i] = velocity_total * sin(angle[i]);
                            obj_vy[i] = -velocity_total * cos(angle[i]);
                        }
                    }
                    else {
                    }
                    break;
                case TRI_RIGHT_EDGE:
                    if (direction[i] == CLOCKWISE) {
                        deviation = obj_x[i] - (tri_center[0] + (tri_len + corner_radius) * cos(pi / 6));
                        if (deviation > 0) {
                            obj_traj_seg[i] = TRI_RIGHT_CORNER;
                            velocity_total = 2 * obj_vx[i];
                            angle[i] = 13 * pi / 6 - velocity_total / corner_radius;
                        }
                    }
                    else {
                    }
                    break;
                default:
                    break;
                }
            case playground:

                break;
            case eight:
                switch (obj_traj_seg[i]) {
                case EIGHT_VER_LINE:
                    if (direction[i] == CLOCKWISE) {
                        velocity_total = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        deviation = velocity_total - (eight_center[1] + eight_len - obj_y[i]);
                        if (deviation > 0) {
                            obj_traj_seg[i] = EIGHT_RIGHT_CIR;
                            angle[i] = pi - deviation / eight_radius;
                            if (angle[i] < 0) {
                                angle[i] += 2 * pi;
                            }
                            obj_x[i] = eight_center[0] + eight_len + eight_radius * cos(angle[i]);
                            obj_y[i] = eight_center[1] + eight_len + eight_radius * sin(angle[i]);
                            obj_vx[i] = velocity_total * sin(angle[i]);
                            obj_vy[i] = -velocity_total * cos(angle[i]);
                        }
                        else {
                            obj_y[i] += obj_vy[i];
                        }
                    }
                    else {
                    }
                    break;
                case EIGHT_RIGHT_CIR:
                    if (direction[i] == CLOCKWISE) {
                      if (angle[i] < pi) {
                            corner_left = (angle[i] + pi / 2) * eight_radius;
                        }
                        else {
                            corner_left = (angle[i] - 3 * pi / 2) * eight_radius;
                        }
                        velocity_total = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        deviation = velocity_total - corner_left;
                        if (deviation > 0) {
                            obj_traj_seg[i] = EIGHT_PAR_LINE;
                            obj_vx[i] = -velocity_total;
                            obj_vy[i] = 0.0;
                            obj_x[i] = eight_center[0] + eight_len - deviation;
                            obj_y[i] = eight_center[1];
                        }
                        else {
                            angle[i] -= velocity_total / eight_radius;
                            if (angle[i] < 0) {
                                angle[i] += 2 * pi;
                            }
                            obj_x[i] = eight_center[0] + eight_len + eight_radius * cos(angle[i]);
                            obj_y[i] = eight_center[1] + eight_len + eight_radius * sin(angle[i]);
                            obj_vx[i] = velocity_total * sin(angle[i]);
                            obj_vy[i] = -velocity_total * cos(angle[i]);
                        }
                    }
                    else {

                    }
                    break;
                case EIGHT_PAR_LINE:
                    if (direction[i] == CLOCKWISE) {
                        velocity_total = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        deviation = velocity_total - (obj_x[i] - (eight_center[0] - eight_len));
                        if (deviation > 0) {
                            obj_traj_seg[i] = EIGHT_LEFT_CIR;
                            angle[i] = pi / 2 + deviation / eight_radius;
                            obj_x[i] = eight_center[0] - eight_len + eight_radius * cos(angle[i]);
                            obj_y[i] = eight_center[1] - eight_len + eight_radius * sin(angle[i]);
                            obj_vx[i] = -velocity_total * sin(angle[i]);
                            obj_vy[i] = velocity_total * cos(angle[i]);
                        }
                        else {
                            obj_x[i] += obj_vx[i];
                        }
                    }
                    else {
                    }
                    break;
                case EIGHT_LEFT_CIR:
                    if (direction[i] == CLOCKWISE) {
                        corner_left = (2 * pi - angle[i]) * eight_radius;
                        velocity_total = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                        deviation = velocity_total - corner_left;
                        if (deviation > 0) {
                            obj_traj_seg[i] = EIGHT_VER_LINE;
                            obj_x[i] = eight_center[0];
                            obj_y[i] = eight_center[1] - eight_len + deviation;
                            obj_vx[i] = 0.0;
                            obj_vy[i] = velocity_total;
                        }
                        else {
                            angle[i] += velocity_total / eight_radius;
                            obj_x[i] = eight_center[0] - eight_len + eight_radius * cos(angle[i]);
                            obj_y[i] = eight_center[1] - eight_len + eight_radius * sin(angle[i]);
                            obj_vx[i] = -velocity_total * sin(angle[i]);
                            obj_vy[i] = +velocity_total * cos(angle[i]);
                        }
                    }
                    else {
                    }
                default:
                    break;
                }
            default:
                break;
            }
        }

        std::string arg;
        if (argv[1]) {
            arg = std::string(argv[1]);
        }
        if (arg == "-v") {
            print_location_verbose();
        }
        else {
            print_location_concise();
        }

        if (sim_time % TIME_BET_SCHE == 0) {
            scheduler_2();
            for (int i = 0; i < OBJ_NUM; i++) {
                obj_vx[i] = vel_adjust[i][0];
                obj_vy[i] = vel_adjust[i][1];
            }
        }

        int ret = 0;
        ret = collision_detection();
        if (ret == 1) {
            // return 1;
        }
    }

    return 0;
}


// *
// Test whether there exists a pair of cars that have been collided with each other in the current state.
// *
int collision_detection() {
    long double dist_sq = 0;
    int j = 0;
    int k = 0;
    for (j = 0; j < OBJ_NUM - 1; j++) {
        for (k = j + 1; k < OBJ_NUM; k++) {
            dist_sq = pow((obj_x[j] - obj_x[k]), 2) + pow((obj_y[j] - obj_y[k]), 2);
            if (dist_sq < pow((1.0 * (obj_radius[j] + obj_radius[k])), 2)) {
                std::cout << "Car " << j << " collided with car " << k << " at sim_time " << sim_time << "!" << std::endl;
                return 1;
            }
        }
    }
    return 0;
}

void scheduler_1() {
    long double obj_vx_initial[OBJ_NUM] = {0.0};
    long double obj_vy_initial[OBJ_NUM] = {0.0};
    long double obj_x_initial[OBJ_NUM] = {0.0};
    long double obj_y_initial[OBJ_NUM] = {0.0};
    long double obj_x_tmp[OBJ_NUM] = {0.0};
    long double obj_y_tmp[OBJ_NUM] = {0.0};
    long double obj_vx_tmp[OBJ_NUM] = {0.0};
    long double obj_vy_tmp[OBJ_NUM] = {0.0};
    int i = 0;
    int j = 0;
    int k = 0;
    long double dist_sq = 0;
    long double adj_coe = 1.0 - adj_step_size;
    int adjusted_car;
    int is_safe = 1;

    for (i = 0; i < OBJ_NUM; i++) {
        obj_vx_initial[i] = obj_vx[i];
        obj_vy_initial[i] = obj_vy[i];
        obj_x_initial[i] = obj_x[i];
        obj_y_initial[i] = obj_y[i];
        obj_x_tmp[i] = obj_x[i];
        obj_y_tmp[i] = obj_y[i];
        obj_vx_tmp[i] = obj_vx[i];
        obj_vy_tmp[i] = obj_vy[i];
    }

    int success = 0;
    for (i = 0; i < SCHE_INTERVAL; i++) {
        for (j = 0; j < OBJ_NUM; j++) {
            obj_x_tmp[j] += obj_vx_tmp[j];
            obj_y_tmp[j] += obj_vy_tmp[j];
        }
        for (j = 0; j < OBJ_NUM - 1; j++) {
            for (k = j + 1; k < OBJ_NUM; k++) {
                dist_sq = pow((obj_x_tmp[j] - obj_x_tmp[k]), 2) + pow((obj_y_tmp[j] - obj_y_tmp[k]), 2);
                if (dist_sq < pow((safety_coe * (obj_radius[j] + obj_radius[k])), 2)) {
                    long double velocity_j;
                    long double velocity_k;
                    velocity_j = pow(obj_vx_tmp[j], 2) + pow(obj_vy_tmp[j], 2);
                    velocity_k = pow(obj_vx_tmp[k], 2) + pow(obj_vy_tmp[k], 2);
                    is_safe = 0;
                    adj_coe = 1.0 - adj_step_size;
                    if (velocity_j > velocity_k) {
                        // obj_vx_tmp[j] = obj_vx_tmp[j] * ((long double)(i + RATIO * SCHE_INTERVAL) / (long double)((RATIO + 1) * SCHE_INTERVAL));
                        // obj_vy_tmp[j] = obj_vy_tmp[j] * ((long double)(i + RATIO * SCHE_INTERVAL) / (long double)((RATIO + 1) * SCHE_INTERVAL));
                        // velocity_total[j] = sqrt(pow(obj_vx_tmp[j], 2) + pow(obj_vy_tmp[j], 2));
                        for (int l = 0; l < adj_times; l++) {
                            long double j_x = obj_x_initial[j];
                            long double j_y = obj_y_initial[j];
                            long double k_x = obj_x_initial[k];
                            long double k_y = obj_y_initial[k];
                            if (l == adj_times - 1) {
                                adj_coe = 0.0;
                            }
                            long double j_vx = adj_coe * obj_vx_initial[j];
                            long double j_vy = adj_coe * obj_vy_initial[j];
                            long double k_vx = obj_vx_initial[k];
                            long double k_vy = obj_vy_initial[k];
                            success = 1;
                            int dist_sq;
                            for (int m = 0; m < SCHE_INTERVAL; m++) {
                                dist_sq = pow((j_x - k_x), 2) + pow((j_y - k_y), 2);
                                if (dist_sq < pow(((safety_coe + 1.0) * (obj_radius[j] + obj_radius[k])), 2)) {
                                    success = 0;
                                    break;
                                }
                                j_x += j_vx;
                                j_y += j_vy;
                                k_x += k_vx;
                                k_y += k_vy;
                            }
                            if (success == 1) {
                                // if (l == adj_times - 1) {
                                //     adj_coe = 0.0;
                                // }
                                break;
                            }

                            adj_coe = adj_coe - adj_step_size;
                        }
                        if (success == 1) {
                            is_adjust[j] = 1;
                            adjusted_car = j;
                            // std::cout << "Car " << adjusted_car << " is adjusted at time " << sim_time << "." << std::endl;
                            // print_location_verbose();
                            break;
                        }
                    }
                    else {
                        // obj_vx_tmp[j] = obj_vx_tmp[j] * ((long double)(i + RATIO * SCHE_INTERVAL) / (long double)((RATIO + 1) * SCHE_INTERVAL));
                        // obj_vy_tmp[j] = obj_vy_tmp[j] * ((long double)(i + RATIO * SCHE_INTERVAL) / (long double)((RATIO + 1) * SCHE_INTERVAL));
                        // velocity_total[j] = sqrt(pow(obj_vx_tmp[j], 2) + pow(obj_vy_tmp[j], 2));
                        for (int l = 0; l < adj_times; l++) {
                            long double j_x = obj_x_initial[j];
                            long double j_y = obj_y_initial[j];
                            long double k_x = obj_x_initial[k];
                            long double k_y = obj_y_initial[k];

                            long double j_vx = obj_vx_initial[j];
                            long double j_vy = obj_vy_initial[j];
                            long double k_vx = adj_coe * obj_vx_initial[k];
                            long double k_vy = adj_coe * obj_vy_initial[k];
                            success = 1;
                            int dist_sq;
                            for (int m = 0; m < SCHE_INTERVAL; m++) {
                                dist_sq = pow((j_x - k_x), 2) + pow((j_y - k_y), 2);
                                if (dist_sq < pow(((safety_coe + 1.0) * (obj_radius[j] + obj_radius[k])), 2)) {
                                    success = 0;
                                    break;
                                }
                                j_x += j_vx;
                                j_y += j_vy;
                                k_x += k_vx;
                                k_y += k_vy;
                            }
                            if (success == 1) {
                                // if (l == adj_times - 1) {
                                //     adj_coe = 0.0;
                                // }
                                break;
                            }
                            adj_coe = adj_coe - adj_step_size;
                        }
                        if (success == 1) {
                            is_adjust[k] = 1;
                            adjusted_car = k;
                            // std::cout << "Car " << adjusted_car << " is adjusted at time " << sim_time << "." << std::endl;
                            // print_location_verbose();
                            break;
                        }
                    }
                }
            }
            if (success == 1) {
                break;
            }
        }
        if (success == 1) {
            break;
        }
    }

// long double velocity_total_initial = 0.0;
// long double velocity_total_tmp = 0.0;
// for (i = 0; i < OBJ_NUM; i++) {
//     velocity_total_initial = sqrt(pow(obj_vx_initial[i], 2) + pow(obj_vy_initial[i], 2));
//     velocity_total_tmp = sqrt(pow(obj_vx_tmp[i], 2) + pow(obj_vy_tmp[i], 2));
//     vel_adjust[i] = velocity_total_tmp / velocity_total_initial;
// }

    int is_update = 0;
    int is_still_safe = 1;
    int updated_car = 0;
    if (is_safe == 1) {
        for (int n = 0; n < OBJ_NUM; n++) {
            if (is_adjust[n] == 0) {    
                continue;
            }
            else {
                is_still_safe = 1;
                for (i = 0; i < OBJ_NUM; i++) {
                    if (i == n) {
                        obj_x_tmp[i] = obj_x[i];
                        obj_y_tmp[i] = obj_y[i];
                        long double vel_angle;
                        vel_angle = return_velocity_angle(i);
                        obj_vx_tmp[i] = initial_vel[i] * cos(vel_angle);
                        obj_vy_tmp[i] = initial_vel[i] * sin(vel_angle);
                    }
                    else {
                        obj_x_tmp[i] = obj_x[i];
                        obj_y_tmp[i] = obj_y[i];
                        obj_vx_tmp[i] = obj_vx[i];
                        obj_vy_tmp[i] = obj_vy[i];
                    }
                }
                for (i = 0; i < SCHE_INTERVAL; i++) {
                    for (j = 0; j < OBJ_NUM; j++) {
                        obj_x_tmp[j] += obj_vx_tmp[j];
                        obj_y_tmp[j] += obj_vy_tmp[j];
                    }
                    for (j = 0; j < OBJ_NUM - 1; j++) {
                        for (k = j + 1; k < OBJ_NUM; k++) {
                            dist_sq = pow((obj_x_tmp[j] - obj_x_tmp[k]), 2) + pow((obj_y_tmp[j] - obj_y_tmp[k]), 2);
                            if (dist_sq < pow(((safety_coe + 1.0) * (obj_radius[j] + obj_radius[k])), 2)) {
                                is_still_safe = 0;
                            }
                        }
                    }
                }
                if (is_still_safe == 1) {
                    is_update = 1;
                    updated_car = n;
                    is_adjust[n] = 0;
                    // std::cout << "Car " << updated_car << " is updated at time " << sim_time << "." << std::endl;
                    // print_location_verbose();
                    break;
                }
            }
        }
    }

    if (is_safe == 1) {
        if (is_update == 1) {
            for (i = 0; i < OBJ_NUM; i++) {
                if (i == updated_car) {
                    long double vel_angle;
                    vel_angle = return_velocity_angle(i);
                    vel_adjust[i][0] = initial_vel[i] * cos(vel_angle);
                    vel_adjust[i][1] = initial_vel[i] * sin(vel_angle);
                }
                else {
                    vel_adjust[i][0] = obj_vx[i];
                    vel_adjust[i][1] = obj_vy[i];
                }
            }
        }
        else {
            for (i = 0; i < OBJ_NUM; i++) {
                vel_adjust[i][0] = obj_vx[i];
                vel_adjust[i][1] = obj_vy[i];
            }
        }
    }
    else {
        for (i = 0; i < OBJ_NUM; i++) {
            if (i == adjusted_car) {
                vel_adjust[i][0] = adj_coe * obj_vx[i];
                vel_adjust[i][1] = adj_coe * obj_vy[i];
            }
            else {
                vel_adjust[i][0] = obj_vx[i];
                vel_adjust[i][1] = obj_vy[i];
            }
        }
    }

    // for (i = 0; i < OBJ_NUM; i++) {
    //     if (obj_vx[i] > 3.0 || obj_vy[i] > 3.0 || abs(obj_x[i]) > 2000.0 || abs(obj_y[i]) > 2000.0) {
    //         int a = 0;
    //         break;
    //     }
    // }
}

void scheduler_2() {
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

    // *
    // Save the input position vectors and velocity vectors into temporary variables in order not to destroy the global variables.
    // *
    for (i = 0; i < OBJ_NUM; i++) {
        // vx_initial[i] = obj_vx[i];
        // vy_initial[i] = obj_vy[i];
        // x_initial[i] = obj_x[i];
        // y_initial[i] = obj_y[i];
        x_tmp[i] = obj_x[i];
        y_tmp[i] = obj_y[i];
        vx_tmp[i] = obj_vx[i];
        vy_tmp[i] = obj_vy[i];
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


    // *
    // Send the adjusted velocity of each car to main().
    // *
    for (i = 0; i < OBJ_NUM; i++) {
        vel_adjust[i][0] = vx_tmp[i];
        vel_adjust[i][1] = vy_tmp[i];
    }
}

int unsafe_state_detector(int j, int k, long double jx, long double jy, long double jvx, long double jvy, long double kx, long double ky, long double kvx, long double kvy, long double coe) {
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

void print_location_verbose() {
    if (sim_time % PRINT_INTERVAL == 0) {
        for (int i = 0; i < OBJ_NUM; i++) {
            std::cout << "Car " << i << ": " << "x: " << obj_x[i] << ", y: " << obj_y[i] << "." << std::endl;
            std::cout << "Velocity: " << "x: " << obj_vx[i] << ", y: " << obj_vy[i] << "." << std::endl;
        }
        std::cout << "At time " << sim_time << "." << std::endl;
        std::cout << std::endl;
    }
}

// int counter = 0;
void print_location_concise() {
    if (sim_time % PRINT_INTERVAL == 0) {
        for (int i = 0; i < OBJ_NUM; i++) {
            std::cout << obj_x[i] << " " << obj_y[i] << " ";
        }
        // Used to draw with simulation time
        // std::cout << "counter" << counter;
        std::cout << std::endl;
    }
}

long double return_velocity_angle(int id) {
    switch (obj_traj[id]) {
    case rect:
        switch (obj_traj_seg[id]) {
        case UPPER_LEFT:
            if (direction[id] == CLOCKWISE) {
                return angle[id] - pi / 2;
            }
            else {

            }
            break;
        case UPPER_EDGE:
            if (direction[id] == CLOCKWISE) {
                return 0.0;
            }
            else {

            }
            break;
        case UPPER_RIGHT:
            if (direction[id] == CLOCKWISE) {
                return 3 * pi / 2 + angle[id];
            }
            else {

            }
            break;
        case RIGHT_EDGE:
            if (direction[id] == CLOCKWISE) {
                return 3 * pi / 2;
            }
            else {

            }
            break;
        case BOTTOM_RIGHT:
            if (direction[id] == CLOCKWISE) {
                return angle[id] - pi / 2;
            }
            else {

            }
            break;
        case BOTTOM_EDGE:
            if (direction[id] == CLOCKWISE) {
                return pi;
            }
            else {

            }
            break;
        case BOTTOM_LEFT:
            if (direction[id] == CLOCKWISE) {
                return angle[id] - pi / 2;
            }
            else {

            }
            break;
        case LEFT_EDGE:
            if (direction[id] == CLOCKWISE) {
                return pi / 2;
            }
            else {

            }
            break;
        default:
            break;
        }
        break;
    case cir:
        return angle[id];
        break;
    case eight:
        switch (obj_traj_seg[id]) {
        case EIGHT_VER_LINE:
            return pi / 2;
            break;
        case EIGHT_RIGHT_CIR:
            return angle[id] - pi / 2;
            break;
        case EIGHT_PAR_LINE:
            return pi;
            break;
        case EIGHT_LEFT_CIR:
            return angle[id] - pi / 2;
            break;
        default:
            break;
        }
    default:
        break;
    }
}

void initialize_car(int i, traj traj, int traj_seg, long double x, long double y, long double ang, long double vel, int dir) {
    switch (traj) {
    case rect:
        break;
    case cir:
        if (dir == CLOCKWISE) {
            obj_x[i] = cir_center[0] + cir_radius * cos(ang);
            obj_y[i] = cir_center[1] + cir_radius * sin(ang);
            obj_vx[i] = vel * sin(ang);
            obj_vy[i] = vel * cos(ang);
            angle[i] = ang;
            initial_vel[i] = vel;
            direction[i] = dir;
            obj_traj[i] = traj;
            obj_traj_seg[i] = traj_seg;
        }
        break;
    case eight:
        switch (traj_seg) {
        case EIGHT_PAR_LINE:
            if (dir == CLOCKWISE) {
                obj_x[i] = x;
                obj_y[i] = eight_center[1];
                obj_vx[i] = -vel;
                obj_vy[i] = 0.0;
                angle[i] = pi;
                initial_vel[i] = vel;
                direction[i] = dir;
                obj_traj[i] = traj;
                obj_traj_seg[i] = traj_seg;
            }
            break;
        case EIGHT_VER_LINE:
            if (dir == CLOCKWISE) {
                obj_x[i] = eight_center[0];
                obj_y[i] = y;
                obj_vx[i] = 0.0;
                obj_vy[i] = vel;
                angle[i] = pi / 2;
                initial_vel[i] = vel;
                direction[i] = dir;
                obj_traj[i] = traj;
                obj_traj_seg[i] = traj_seg;
            }
            break;
        default:
            break;
        }
    default:
        break;
    }
}