// bka_dynamic_tb.cpp  — C-simulation testbench
// Compile with: g++ -DBKA_NATIVE_FLOAT_CSIM bka_dynamic_tb.cpp bka_dynamic.cpp -o sim -lm
// (BKA_NATIVE_FLOAT_CSIM enables the float typedef path for desktop compilation)

#include "bka_dynamic.h"
#include <chrono>
#include <cmath>
#include <iostream>

static bool validate_path(
    const data_t path_x[MAX_ITER],
    const data_t path_y[MAX_ITER],
    int  iterations,
    const data_t obstacles[MAX_OBS * 3],
    int  obstacle_count,
    data_t goal_x, data_t goal_y,
    int  reached)
{
    if (iterations < 1 || iterations > MAX_ITER) {
        std::cout << "FAIL: invalid iteration count " << iterations << "\n";
        return false;
    }
    int    collisions = 0;
    double length     = 0.0;
    for (int t = 0; t < iterations; ++t) {
        double px = (double)path_x[t];
        double py = (double)path_y[t];
        if (t != 0) {
            double dx = px - (double)path_x[t-1];
            double dy = py - (double)path_y[t-1];
            length += std::sqrt(dx*dx + dy*dy);
        }
        for (int i = 0; i < obstacle_count; ++i) {
            double dx = px - (double)obstacles[i*3];
            double dy = py - (double)obstacles[i*3+1];
            double r  = (double)obstacles[i*3+2];
            if (std::sqrt(dx*dx + dy*dy) < r) {
                std::cout << "COLLISION: step=" << t << " obs=" << i << "\n";
                ++collisions;
            }
        }
    }
    double fx = (double)path_x[iterations-1];
    double fy = (double)path_y[iterations-1];
    double err = std::sqrt((goal_x-fx)*(goal_x-fx) + (goal_y-fy)*(goal_y-fy));

    std::cout << "Iterations  : " << iterations << " / " << MAX_ITER << "\n";
    std::cout << "Goal reached: " << (reached ? "YES" : "NO") << "\n";
    std::cout << "Final pos   : (" << fx << ", " << fy << ")\n";
    std::cout << "Goal error  : " << err << " px\n";
    std::cout << "Path length : " << length << " px\n";
    std::cout << "Collisions  : " << collisions << "\n";

    bool passed = reached && err < 6.0 && collisions == 0;
    std::cout << (passed ? "ALL TESTS PASSED\n" : "TEST FAILED\n");
    return passed;
}

static bool run_case(
    const char *name,
    data_t start_x, data_t start_y,
    data_t goal_x, data_t goal_y,
    const data_t raw[MAX_OBS][3],
    int n_obs)
{
    data_t obs[MAX_OBS * 3] = {};
    for (int i = 0; i < n_obs; ++i) {
        obs[i*3] = raw[i][0]; obs[i*3+1] = raw[i][1]; obs[i*3+2] = raw[i][2];
    }

    data_t px[MAX_ITER] = {}, py[MAX_ITER] = {};
    int iters = 0, flag = 0;

    std::cout << "\n=== " << name << " ===\n";
    std::cout << "Start: (" << start_x << ", " << start_y << ")  "
              << "Goal: ("  << goal_x  << ", " << goal_y  << ")\n";

    auto t0 = std::chrono::high_resolution_clock::now();
    bka_hw_engine(start_x, start_y, goal_x, goal_y,
                  obs, n_obs, px, py, &iters, &flag);
    auto t1 = std::chrono::high_resolution_clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    std::cout << "Host C-sim time: " << us << " us\n";

    return validate_path(px, py, iters, obs, n_obs, goal_x, goal_y, flag);
}

int main()
{
    std::cout << "IMBKA + map-aware vortex C-simulation\n";

    const data_t static_map[MAX_OBS][3] = {
        {120.0f, 240.0f, 30.0f},
        {180.0f, 180.0f, 54.0f},
        {180.0f, 420.0f, 60.0f},
        {300.0f,  60.0f, 48.0f},
        {360.0f, 480.0f, 30.0f},
        {420.0f, 240.0f, 42.0f},
        {480.0f, 120.0f, 42.0f}
    };

    const data_t clustered_map[MAX_OBS][3] = {
        { 55.0f, 270.0f, 95.0f},
        { 55.0f, 210.0f, 42.0f},
        {150.0f, 240.0f, 64.0f}
    };

    const data_t right_corridor_map[MAX_OBS][3] = {
        {165.0f, 165.0f, 58.0f},
        {115.0f, 225.0f, 58.0f},
        {235.0f, 115.0f, 38.0f}
    };

    bool ok = true;
    ok = run_case("7-obstacle static map",
                  30.0f, 30.0f, 420.0f, 420.0f,
                  static_map, 7) && ok;
    ok = run_case("clustered map from plotted failure shape",
                  30.0f, 30.0f, 420.0f, 420.0f,
                  clustered_map, 3) && ok;
    ok = run_case("blocked-left/right-corridor map",
                  30.0f, 30.0f, 430.0f, 360.0f,
                  right_corridor_map, 3) && ok;

    return ok ? 0 : 1;
}
