#include <vector>
#include <iostream>
#include <cmath>
#include <string>

// sim_time unit: ms
// distance unit: mm
// velocity unit: mm / ms

const long double pi = 3.14159265358979323846;
const long double adj_step_size = 0.1;
const int adj_times = 8;

#define OBJ_NUM       3


#define TIME_BET_SCHE 50

int is_adjust[OBJ_NUM] = {0};
long double initial_vel[OBJ_NUM];
// used by scheduler 1
// #define RATIO         5
#define SCHE_INTERVAL 10
// used by scheduler 1

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

#define CLOCKWISE           0
#define COUNTER_CLOCKWISE   1

#define PRINT_INTERVAL      200

long double obj_radius[OBJ_NUM] = {200.0, 200.0, 200.0};
long double obj_x     [OBJ_NUM];
long double obj_y     [OBJ_NUM];
long double obj_vx    [OBJ_NUM];
long double obj_vy    [OBJ_NUM];
long double vel_adjust[OBJ_NUM][2];
long double safety_coe = 1.5;

long double corner_radius = 200.0;

long double rect_center[2] = {0.0, 0.0};
long double rect_len = 1000.0;

long double cir_center[2] = {0.0, 0.0};
long double cir_radius = 1200.0;

long double tri_center[2];
long double tri_len;

long double angle[OBJ_NUM];

int direction[OBJ_NUM];

int simulation_time = 1000000000;
int sim_time = 0;

enum traj {rect, cir, tri, playground};
traj obj_traj[OBJ_NUM];
int obj_traj_seg[OBJ_NUM];

void scheduler_1();
void scheduler_2();
int collision_detection();
void print_location_verbose();
void print_location_concise();
long double return_velocity_angle(int id);

int main(int argc, char **argv) {
    obj_x[0] = rect_center[0] - rect_len - corner_radius; 
    obj_y[0] = rect_center[1];
    initial_vel[0] = 0.1;
    obj_vx[0] = 0.0;
    obj_vy[0] = 0.1;
    obj_traj[0] = rect;
    obj_traj_seg[0] = LEFT_EDGE;
    direction[0] = CLOCKWISE;
    angle[0] = 0;

    obj_x[1] = cir_center[0];
    obj_y[1] = cir_center[1] + cir_radius;
    initial_vel[1] = 0.2;
    obj_vx[1] = 0.2;
    obj_vy[1] = 0.0;
    obj_traj[1] = cir;
    obj_traj_seg[1] = 0;
    direction[1] = CLOCKWISE;
    angle[1] = pi / 2;

    obj_x[2] = cir_center[0];
    obj_y[2] = cir_center[1] - cir_radius;
    initial_vel[2] = 0.3;
    obj_vx[2] = -0.3;
    obj_vy[2] = 0.0;
    obj_traj[2] = cir;
    obj_traj_seg[2] = 0;
    direction[2] = CLOCKWISE;
    angle[2] = 3 * pi / 2;

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
                    else { // TO BE DONE
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
                    else { // TO BE DONE
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
                    else { // TO BE DONE
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
                    else { // TO BE DONE
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
                    else { // TO BE DONE
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
                    else { // TO BE DONE
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
                    else { // TO BE DONE
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
                    else { // TO BE DONE
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
                    if (angle[i] > 2 * pi) {
                        angle[i] = angle[i] - 2 * pi;
                    }
                    obj_x[i] = cir_center[0] + cir_radius * cos(angle[i]);
                    obj_y[i] = cir_center[1] + cir_radius * sin(angle[i]);
                    obj_vx[i] = velocity_total * sin(angle[i]);
                    obj_vy[i] = -velocity_total * cos(angle[i]);
                }
                else { // TO BE DONE
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
                    else { // TO BE DONE
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
                    else { // TO BE DONE
                    }
                    break;
                default:
                    break;
                }
            case playground:
                
                break;
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
            scheduler_1();
            for (int i = 0; i < OBJ_NUM; i++) {
                obj_vx[i] = vel_adjust[i][0];
                obj_vy[i] = vel_adjust[i][1];
            }
        }

        int ret = 0;
        // ret = collision_detection();
        if (ret == 1) {
            std::cout << "sim_time " << sim_time << "." << std::endl;
            return 1;
        }
    }

    return 0;
}

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
    // long double velocity_total[OBJ_NUM] = {0.0};
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
                            
                            long double j_vx = adj_coe * obj_vx_initial[j];
                            long double j_vy = adj_coe * obj_vy_initial[j];
                            long double k_vx = obj_vx_initial[k];
                            long double k_vy = obj_vy_initial[k];
                            for (int m = 0; m < SCHE_INTERVAL; m++) {
                                success = 1;
                                int dist_sq;
                                dist_sq = pow((j_x - k_x), 2) + pow((j_y - k_y), 2);
                                if (dist_sq < pow((safety_coe * (obj_radius[j] + obj_radius[k])), 2)) {
                                    success = 0;
                                    break;
                                }
                                j_x += j_vx;
                                j_y += j_vy;
                                k_x += k_vx;
                                k_y += k_vy;
                            }
                            if (success == 1) {
                                break;
                            }
                            adj_coe = adj_coe - adj_step_size;
                        }
                        if (success == 1) {
                            is_adjust[j] = 1;
                            adjusted_car = j;
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
                            for (int m = 0; m < SCHE_INTERVAL; m++) {
                                success = 1;
                                int dist_sq;
                                dist_sq = pow((j_x - k_x), 2) + pow((j_y - k_y), 2);
                                if (dist_sq < pow((safety_coe * (obj_radius[j] + obj_radius[k])), 2)) {
                                    success = 0;
                                    break;
                                }
                                j_x += j_vx;
                                j_y += j_vy;
                                k_x += k_vx;
                                k_y += k_vy;
                            }
                            if (success == 1) {
                                break;
                            }
                            adj_coe = adj_coe - adj_step_size;
                        }
                        if (success == 1) {
                            is_adjust[k] = 1;
                            adjusted_car = k;
                            break;
                        }
                    }
                }
            }
            if (success == 1) {
                break;
            }
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
                            if (dist_sq < pow((safety_coe * (obj_radius[j] + obj_radius[k])), 2)) {
                                is_still_safe = 0;
                            }
                        }
                    }
                }
                if (is_still_safe = 1) {
                    is_update = 1;
                    updated_car = n;
                    is_adjust[n] = 0;
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
                vel_adjust[i][0] = (adj_coe - adj_step_size) * obj_vx[i];
                vel_adjust[i][1] = (adj_coe - adj_step_size) * obj_vy[i];
            }
            else {
                vel_adjust[i][0] = obj_vx[i];
                vel_adjust[i][1] = obj_vy[i];
            }
        }
    }
}


void scheduler_2() {
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

void print_location_concise() {
    if (sim_time % PRINT_INTERVAL == 0) {
        for (int i = 0; i < OBJ_NUM; i++) {
            std::cout << obj_x[i] << " " << obj_y[i] << " ";
        }
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
    default:
        break;
    }
}