#ifndef BKA_DYNAMIC_H
#define BKA_DYNAMIC_H



#ifdef BKA_NATIVE_FLOAT_CSIM
typedef float coord_t;
typedef float unit_t;
typedef float angle_t;
typedef float dist2_t;
typedef float fit_t;
#else
#include "ap_fixed.h"
typedef ap_fixed<18, 11,  AP_TRN, AP_SAT>  coord_t;
typedef ap_fixed<16,  3,  AP_TRN, AP_SAT>  unit_t;
typedef ap_fixed<18,  5,  AP_TRN, AP_SAT>  angle_t;
typedef ap_ufixed<36, 20, AP_TRN, AP_WRAP> dist2_t;
typedef ap_ufixed<40, 22, AP_TRN, AP_WRAP> fit_t;
#endif

typedef float data_t;

#define MAX_ITER   800
#define MAX_OBS     20
#define HIST_SIZE   64
#define N_KITES      4

struct Obstacle {
    coord_t x;
    coord_t y;
    coord_t radius;
};

void bka_hw_engine(
    data_t  start_x,
    data_t  start_y,
    data_t  goal_x,
    data_t  goal_y,
    data_t  obs_data[MAX_OBS * 3],
    int     num_obstacles,
    data_t  path_out_x[MAX_ITER],
    data_t  path_out_y[MAX_ITER],
    int    *iterations_used,
    int    *goal_reached_flag
);

#endif // BKA_DYNAMIC_H
