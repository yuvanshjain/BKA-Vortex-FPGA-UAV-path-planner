// =============================================================================
//  bka_dynamic.cpp  -  IMBKA + Vortex UAV path planner
//  Target: xc7z020 (PYNQ-Z2)
//  Budget: <= 50k LUTs, POSITIVE SLACK at 20ns period (50 MHz)
//
//  FINAL TIMING FIXES:
//  1. Eradicated 'fpext' (Float Extension) operations from the critical path
//     by exclusively using native ap_ufixed hls::sqrt routines.
//  2. Injected #pragma HLS LATENCY min=X into bottleneck functions
//  3. Forced sequential loop execution (PIPELINE off, unroll factor=1) to
//     crush the MUX footprint back down to ~35k-45k LUTs.
//
//  MATH FIX: Isolated Trig functions to 24-bit precision to prevent 6.28 wrap-around
//
//  PATH FIX (this revision): keep the IMBKA attack/migration math and
//  BKA-vortex blend intact, but replace the local left/right obstacle
//  vote with a small map-aware corridor cost. On vortex entry, and only
//  occasionally while evading, the planner samples both tangential
//  corridors against all obstacles, walls, and distance-to-goal. The
//  selected vortex_sign is therefore the lower-cost route through the
//  map, not a hard-coded left turn. Re-votes keep hysteresis so the UAV
//  does not chatter between clockwise and counter-clockwise arcs.
// =============================================================================

#undef __GMP_LIBGMP_DLL

#include "bka_dynamic.h"

#ifdef BKA_NATIVE_FLOAT_CSIM
#include <cmath>
namespace hls {
    inline float sqrt(float x) { return ::sqrtf(x); }
}
typedef float calc_t;
#else
#include "ap_fixed.h"
#include "hls_math.h"
typedef ap_fixed<24, 8> calc_t;
#endif

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static const coord_t LOOKAHEAD_MARGIN  = (coord_t)25;
static const coord_t VISUAL_PADDING    = (coord_t)8;
static const coord_t MAX_STEP          = (coord_t)6;
static const coord_t ROBOT_RADIUS      = (coord_t)2;
static const coord_t MAP_LIMIT         = (coord_t)500;
static const coord_t WALL_RADAR        = (coord_t)15;
static const coord_t LOOKAHEAD_PX      = (coord_t)72;
static const coord_t MAX_INPUT_RADIUS  = (coord_t)200;
static const coord_t CORRIDOR_STEP     = (coord_t)28;
static const coord_t CORRIDOR_FWD      = (coord_t)14;
static const int     CORRIDOR_SAMPLES  = 4;
static const unit_t  INERTIA_FREE      = (unit_t)0.80f;
static const unit_t  INERTIA_EVADE     = (unit_t)0.35f;
static const dist2_t GOAL_RADIUS_SQ    = (dist2_t)36;
static const unit_t  BETA_STAR         = (unit_t)0.1f;
static const unit_t  SPIRAL_B          = (unit_t)1.0f;

// ---------------------------------------------------------------------------
// Math Helpers
// ---------------------------------------------------------------------------
static inline dist2_t square_distance(coord_t x, coord_t y)
{
    #pragma HLS INLINE
    return (dist2_t)((dist2_t)(x * x) + (dist2_t)(y * y));
}

static coord_t clamp_coord(coord_t v)
{
    #pragma HLS INLINE off
    #pragma HLS LATENCY min=1
    if (v < (coord_t)10)  return (coord_t)10;
    if (v > (coord_t)490) return (coord_t)490;
    return v;
}

// ---------------------------------------------------------------------------
// RNG Functions
// ---------------------------------------------------------------------------
static unsigned int g_rng = 0xDEADBEEFu;

static unit_t rng_f01()
{
    #pragma HLS INLINE off
    unsigned int x = g_rng;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    g_rng = x;

    unsigned int m = (x >> 8) & 0x1FFFu;
    unit_t f = 0;

#ifdef BKA_NATIVE_FLOAT_CSIM
    f = (unit_t)m * (unit_t)(1.0f / 8192.0f);
#else
    f.range(12, 0) = (ap_uint<13>)m;
#endif

    if (f < (unit_t)0.001f) f = (unit_t)0.001f;
    return f;
}

static unit_t rng_cauchy_cheap()
{
    #pragma HLS INLINE off
    unit_t v  = (unit_t)2 * rng_f01() - (unit_t)1;
    unit_t av = v;
    if (v < (unit_t)0) av = -v;

    unit_t c;
    if (av > (unit_t)0.85f) {
        c = (unit_t)(v * (unit_t)5);
    } else {
        c = (unit_t)(v * (unit_t)0.75f);
    }

    if (c >  (unit_t)3) c =  (unit_t)3;
    if (c < -(unit_t)3) c = -(unit_t)3;
    return c;
}

// ---------------------------------------------------------------------------
// Trigonometry Approximations
// ---------------------------------------------------------------------------
static calc_t exp_approx(calc_t x)
{
    #pragma HLS INLINE off
    if (x >  (calc_t)2) x =  (calc_t)2;
    if (x < -(calc_t)2) x = -(calc_t)2;
    calc_t x2 = x * x;
    calc_t x3 = x2 * x;
    return (calc_t)1 + x + (x2 * (calc_t)0.5f) + (x3 * (calc_t)0.16667f);
}

static calc_t sin_approx(calc_t x)
{
    #pragma HLS INLINE off
    const calc_t PI  = (calc_t)3.14159f;
    const calc_t PI2 = (calc_t)6.28318f;

    if (x >  PI) x -= PI2;
    if (x >  PI) x -= PI2;
    if (x < -PI) x += PI2;
    if (x < -PI) x += PI2;

    calc_t sign = (calc_t)1;
    if (x < (calc_t)0) { x = -x; sign = -(calc_t)1; }
    calc_t t     = x * (PI - x);
    calc_t denom = (calc_t)49.348f - (calc_t)4 * t;
    if (denom < (calc_t)0.001f) denom = (calc_t)0.001f;

    return sign * (calc_t)16 * t / denom;
}

static calc_t cos_approx(calc_t x)
{
    #pragma HLS INLINE off
    return sin_approx(x + (calc_t)1.5708f);
}

// ---------------------------------------------------------------------------
// Vector Operations
// ---------------------------------------------------------------------------
static coord_t sqrt_dist(dist2_t d2)
{
    #pragma HLS INLINE off
    #pragma HLS LATENCY min=2
#ifdef BKA_NATIVE_FLOAT_CSIM
    return (coord_t)std::sqrt((float)d2);
#else
    return (coord_t)hls::sqrt(d2);
#endif
}

static unit_t levy_cheap(unit_t rt)
{
    #pragma HLS INLINE off
    #pragma HLS LATENCY min=2
    unit_t alpha = (unit_t)0.05f - (unit_t)0.04f * rt;
    if (alpha < (unit_t)0.01f) alpha = (unit_t)0.01f;
    unit_t mu = rng_f01() * (unit_t)2 - (unit_t)1;
    unit_t nu = rng_f01();
    if (nu < (unit_t)0.05f) nu = (unit_t)0.05f;

    coord_t nu_sq = 0;
#ifdef BKA_NATIVE_FLOAT_CSIM
    nu_sq = (unit_t)std::sqrt((float)nu);
#else
    ap_ufixed<16, 3> nu_u = (ap_ufixed<16, 3>)nu;
    nu_sq = sqrt_dist((dist2_t)nu_u);
#endif

    if (nu_sq < (unit_t)0.001f) nu_sq = (unit_t)0.001f;
    unit_t lf = mu * (unit_t)0.7082f / (unit_t)nu_sq;
    if (lf >  (unit_t)3) lf =  (unit_t)3;
    if (lf < -(unit_t)3) lf = -(unit_t)3;
    return alpha * lf;
}

static void normalize_pair(
    unit_t  x,   unit_t  y,
    unit_t  fbx, unit_t  fby,
    unit_t &ox,  unit_t &oy)
{
    #pragma HLS INLINE off
    #pragma HLS LATENCY min=3
#ifdef BKA_NATIVE_FLOAT_CSIM
    float fx = (float)x, fy = (float)y;
    float mag = std::sqrt(fx * fx + fy * fy);
    if (mag < 0.001f) { ox = fbx; oy = fby; }
    else { ox = (unit_t)(fx / mag); oy = (unit_t)(fy / mag); }
#else
    dist2_t mag_sq = (dist2_t)(x*x + y*y);
    coord_t mag = sqrt_dist(mag_sq);
    if (mag < (coord_t)0.001f) { ox = fbx; oy = fby; }
    else { ox = (unit_t)(x / mag); oy = (unit_t)(y / mag); }
#endif
}

// ---------------------------------------------------------------------------
// Cost Evaluation
// ---------------------------------------------------------------------------
static fit_t imbka_fit(
    unit_t  kx,   unit_t  ky,
    coord_t cx,   coord_t cy,
    coord_t gx,   coord_t gy,
    const Obstacle obs[MAX_OBS],
    int n)
{
    #pragma HLS INLINE off
    #pragma HLS LATENCY min=2
    coord_t wx = cx + (coord_t)((coord_t)kx * LOOKAHEAD_PX);
    coord_t wy = cy + (coord_t)((coord_t)ky * LOOKAHEAD_PX);
    coord_t dgx = gx - wx, dgy = gy - wy;
    fit_t goal_cost = (fit_t)square_distance(dgx, dgy);

    fit_t obs_cost = (fit_t)0;
    FIT_OBS: for (int i = 0; i < n; i++) {
        #pragma HLS PIPELINE off
        #pragma HLS unroll factor=1
        #pragma HLS LOOP_TRIPCOUNT min=0 max=20
        coord_t dx = wx - obs[i].x, dy = wy - obs[i].y;
        dist2_t dsq = square_distance(dx, dy);
        coord_t danger = obs[i].radius + ROBOT_RADIUS + VISUAL_PADDING + LOOKAHEAD_MARGIN;
        dist2_t danger_sq = (dist2_t)(danger * danger);
        if (dsq < danger_sq) {
            dist2_t pen = danger_sq - dsq;
            obs_cost = obs_cost + (fit_t)(pen * (coord_t)0.25f);
        }
    }

    fit_t wall_cost = (fit_t)0;
    coord_t wm = WALL_RADAR + (coord_t)5;
    coord_t d = 0;
    if (wx < wm)           { d = wm - wx;             wall_cost = wall_cost + (fit_t)(d * d * (coord_t)4); }
    if (wx > MAP_LIMIT-wm) { d = wx - (MAP_LIMIT-wm); wall_cost = wall_cost + (fit_t)(d * d * (coord_t)4); }
    if (wy < wm)           { d = wm - wy;             wall_cost = wall_cost + (fit_t)(d * d * (coord_t)4); }
    if (wy > MAP_LIMIT-wm) { d = wy - (MAP_LIMIT-wm); wall_cost = wall_cost + (fit_t)(d * d * (coord_t)4); }

    return goal_cost + obs_cost + wall_cost;
}

static fit_t corridor_cost(
    coord_t dir_x, coord_t dir_y,
    unit_t  goal_dir_x, unit_t goal_dir_y,
    coord_t cx, coord_t cy,
    coord_t gx, coord_t gy,
    const Obstacle obs[MAX_OBS],
    int n,
    coord_t side_threat)
{
    #pragma HLS INLINE off
    #pragma HLS LATENCY min=2

    fit_t cost = (fit_t)(side_threat * (coord_t)96);

    CORRIDOR_SAMPLE: for (int s = 1; s <= CORRIDOR_SAMPLES; s++) {
        #pragma HLS PIPELINE off
        #pragma HLS unroll factor=1
        #pragma HLS LOOP_TRIPCOUNT min=4 max=4
        coord_t ss = (coord_t)s;
        coord_t px = cx + dir_x * CORRIDOR_STEP * ss + (coord_t)goal_dir_x * CORRIDOR_FWD * ss;
        coord_t py = cy + dir_y * CORRIDOR_STEP * ss + (coord_t)goal_dir_y * CORRIDOR_FWD * ss;

        coord_t dgx = gx - px;
        coord_t dgy = gy - py;
        cost = cost + (fit_t)(square_distance(dgx, dgy) * (coord_t)0.03125f);

        coord_t wm = WALL_RADAR + (coord_t)5;
        coord_t wd = 0;
        if (px < wm)           { wd = wm - px;             cost = cost + (fit_t)(wd * wd * (coord_t)12); }
        if (px > MAP_LIMIT-wm) { wd = px - (MAP_LIMIT-wm); cost = cost + (fit_t)(wd * wd * (coord_t)12); }
        if (py < wm)           { wd = wm - py;             cost = cost + (fit_t)(wd * wd * (coord_t)12); }
        if (py > MAP_LIMIT-wm) { wd = py - (MAP_LIMIT-wm); cost = cost + (fit_t)(wd * wd * (coord_t)12); }

        CORRIDOR_OBS: for (int i = 0; i < n; i++) {
            #pragma HLS PIPELINE off
            #pragma HLS unroll factor=1
            #pragma HLS LOOP_TRIPCOUNT min=0 max=20
            coord_t ox = px - obs[i].x;
            coord_t oy = py - obs[i].y;
            coord_t danger = obs[i].radius + ROBOT_RADIUS + VISUAL_PADDING + (coord_t)8;
            dist2_t danger_sq = (dist2_t)(danger * danger);
            dist2_t dsq = square_distance(ox, oy);
            if (dsq < danger_sq) {
                cost = cost + (fit_t)((danger_sq - dsq) * (coord_t)4);
            }
        }
    }

    coord_t align = dir_x * (coord_t)goal_dir_x + dir_y * (coord_t)goal_dir_y;
    if (align < (coord_t)0) {
        cost = cost + (fit_t)((-align) * (coord_t)128);
    }

    return cost;
}

// ---------------------------------------------------------------------------
// Hardware Engine Interface
// ---------------------------------------------------------------------------
void bka_hw_engine(
    data_t  start_x,  data_t  start_y,
    data_t  goal_x,   data_t  goal_y,
    data_t  obs_data[MAX_OBS * 3],
    int     num_obstacles,
    data_t  path_out_x[MAX_ITER],
    data_t  path_out_y[MAX_ITER],
    int    *iterations_used,
    int    *goal_reached_flag)
{
    #pragma HLS INTERFACE s_axilite port=return             bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=start_x            bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=start_y            bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=goal_x             bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=goal_y             bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=num_obstacles       bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=iterations_used    bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=goal_reached_flag  bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=obs_data           bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=path_out_x         bundle=CTRL
    #pragma HLS INTERFACE s_axilite port=path_out_y         bundle=CTRL

    #pragma HLS INTERFACE m_axi port=obs_data    depth=60   offset=slave bundle=IN_BUS
    #pragma HLS INTERFACE m_axi port=path_out_x  depth=800  offset=slave bundle=OUT_BUS
    #pragma HLS INTERFACE m_axi port=path_out_y  depth=800  offset=slave bundle=OUT_BUS

    #pragma HLS ALLOCATION function instances=imbka_fit      limit=1
    #pragma HLS ALLOCATION function instances=corridor_cost  limit=1
    #pragma HLS ALLOCATION function instances=normalize_pair limit=1
    #pragma HLS ALLOCATION function instances=sqrt_dist      limit=1
    #pragma HLS ALLOCATION operation  instances=sdiv         limit=2
    #pragma HLS ALLOCATION operation  instances=udiv         limit=2

    coord_t sx = clamp_coord((coord_t)start_x);
    coord_t sy = clamp_coord((coord_t)start_y);
    coord_t gx = clamp_coord((coord_t)goal_x);
    coord_t gy = clamp_coord((coord_t)goal_y);

    int safe_n = num_obstacles;
    if (safe_n < 0)       safe_n = 0;
    if (safe_n > MAX_OBS) safe_n = MAX_OBS;

    data_t local_obs_raw[MAX_OBS * 3];
    #pragma HLS BIND_STORAGE variable=local_obs_raw type=ram_1p impl=bram

    AXI_BURST_READ: for (int i = 0; i < safe_n * 3; i++) {
        #pragma HLS PIPELINE II=1
        local_obs_raw[i] = obs_data[i];
    }

    Obstacle local_obs[MAX_OBS];
    #pragma HLS BIND_STORAGE variable=local_obs type=ram_1p impl=bram

    LOAD_OBS: for (int i = 0; i < safe_n; i++) {
        #pragma HLS PIPELINE off
        #pragma HLS unroll factor=1
        coord_t ox = (coord_t)local_obs_raw[i*3+0];
        coord_t oy = (coord_t)local_obs_raw[i*3+1];
        coord_t r  = (coord_t)local_obs_raw[i*3+2];
        if (ox < (coord_t)0)      ox = (coord_t)0;
        if (ox > MAP_LIMIT)       ox = MAP_LIMIT;
        if (oy < (coord_t)0)      oy = (coord_t)0;
        if (oy > MAP_LIMIT)       oy = MAP_LIMIT;
        if (r  < (coord_t)1)      r  = (coord_t)1;
        if (r  > MAX_INPUT_RADIUS) r  = MAX_INPUT_RADIUS;
        local_obs[i].x = ox; local_obs[i].y = oy; local_obs[i].radius = r;
    }

    coord_t hist_x[HIST_SIZE];
    coord_t hist_y[HIST_SIZE];
    #pragma HLS BIND_STORAGE variable=hist_x type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=hist_y type=ram_2p impl=bram

    INIT_HIST: for (int i = 0; i < HIST_SIZE; i++) {
        #pragma HLS LOOP_FLATTEN off
        hist_x[i] = sx; hist_y[i] = sy;
    }

    unit_t kite_kx[N_KITES];
    unit_t kite_ky[N_KITES];
    fit_t  kite_fit[N_KITES];
    #pragma HLS ARRAY_PARTITION variable=kite_kx type=complete dim=1
    #pragma HLS ARRAY_PARTITION variable=kite_ky type=complete dim=1
    #pragma HLS ARRAY_PARTITION variable=kite_fit type=complete dim=1

    {
        coord_t fdx = gx - sx, fdy = gy - sy;
        coord_t fd  = sqrt_dist(square_distance(fdx, fdy));
        if (fd < (coord_t)0.001f) fd = (coord_t)0.001f;

#ifdef BKA_NATIVE_FLOAT_CSIM
        unit_t cb = (unit_t)((float)fdx / (float)fd);
        unit_t sb = (unit_t)((float)fdy / (float)fd);
#else
        unit_t cb = (unit_t)(fdx / fd);
        unit_t sb = (unit_t)(fdy / fd);
#endif

        kite_kx[0] =  cb; kite_ky[0] =  sb;
        kite_kx[1] = -sb; kite_ky[1] =  cb;
        kite_kx[2] = -cb; kite_ky[2] = -sb;
        kite_kx[3] =  sb; kite_ky[3] = -cb;
        kite_fit[0] = kite_fit[1] = kite_fit[2] = kite_fit[3] = (fit_t)4194303;
    }

    data_t local_px[MAX_ITER];
    data_t local_py[MAX_ITER];
    #pragma HLS BIND_STORAGE variable=local_px type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=local_py type=ram_2p impl=bram

    coord_t curr_x = sx, curr_y = sy;
    coord_t vel_x  = (coord_t)0, vel_y = (coord_t)0;
    unit_t  vortex_sign  = (unit_t)1;
    int     frustration  = 0;
    int     hist_slot    = 0;
    bool    in_vortex    = false;
    int     actual_iters = MAX_ITER;
    int     reached      = 0;

#ifndef BKA_NATIVE_FLOAT_CSIM
    ap_uint<14> rt_raw       = 0;
    ap_uint<5>  rt_remainder = 0;
#endif

    // ---------------------------------------------------------------------------
    // Main Control Loop
    // ---------------------------------------------------------------------------
    MAIN_LOOP: for (int t = 0; t < MAX_ITER; t++) {
        #pragma HLS PIPELINE off
        #pragma HLS LOOP_TRIPCOUNT min=1 max=800
        #pragma HLS LOOP_FLATTEN off

        coord_t dx_g  = gx - curr_x;
        coord_t dy_g  = gy - curr_y;
        dist2_t dg_sq = square_distance(dx_g, dy_g);

        if (dg_sq < GOAL_RADIUS_SQ) {
            local_px[t]  = (data_t)gx;
            local_py[t]  = (data_t)gy;
            actual_iters = t + 1;
            reached      = 1;
            break;
        }

        coord_t dtg = sqrt_dist(dg_sq);
        if (dtg < (coord_t)0.001f) dtg = (coord_t)0.001f;

#ifdef BKA_NATIVE_FLOAT_CSIM
        unit_t raw_x = (unit_t)((float)dx_g / (float)dtg);
        unit_t raw_y = (unit_t)((float)dy_g / (float)dtg);
        unit_t rt = (unit_t)((float)t / (float)MAX_ITER);
#else
        unit_t raw_x = (unit_t)(dx_g / dtg);
        unit_t raw_y = (unit_t)(dy_g / dtg);
        unit_t rt = 0;
        rt.range(12, 0) = (ap_uint<13>)rt_raw;
#endif

        calc_t safe_rt = (calc_t)rt;
        calc_t a1    = -(calc_t)1 - safe_rt;
        calc_t l_val = (a1 - (calc_t)1) * (calc_t)rng_f01() + (calc_t)1;
        calc_t spiral_arg = (calc_t)6.28318f * l_val;

        unit_t coeff = (unit_t)((calc_t)BETA_STAR * exp_approx((calc_t)SPIRAL_B * l_val) * cos_approx(spiral_arg));

        unit_t levy  = levy_cheap(rt);
        unit_t m_val = (unit_t)((calc_t)2 * sin_approx((calc_t)rng_f01() + (calc_t)1.5708f));

        fit_t best_fit  = (fit_t)4194303;
        fit_t worst_fit = (fit_t)0;
        int   best_k    = 0;
        int   worst_k   = 0;

        EVAL_KITES: for (int k = 0; k < N_KITES; k++) {
            #pragma HLS PIPELINE off
            #pragma HLS unroll factor=1
            #pragma HLS LOOP_TRIPCOUNT min=4 max=4
            fit_t fk = imbka_fit(kite_kx[k], kite_ky[k], curr_x, curr_y, gx, gy, local_obs, safe_n);
            kite_fit[k] = fk;
            if (fk < best_fit)  { best_fit  = fk; best_k  = k; }
            if (fk > worst_fit) { worst_fit = fk; worst_k = k; }
        }

        unit_t leader_x = kite_kx[best_k];
        unit_t leader_y = kite_ky[best_k];

        ATTACK_RNG: for (int k = 0; k < N_KITES; k++) {
            #pragma HLS PIPELINE off
            #pragma HLS unroll factor=1
            #pragma HLS LOOP_TRIPCOUNT min=4 max=4
            (void)rng_f01();
        }

        ALERT: for (int k = 0; k < N_KITES; k++) {
            #pragma HLS PIPELINE off
            #pragma HLS unroll factor=1
            #pragma HLS LOOP_TRIPCOUNT min=4 max=4
            unit_t nx, ny;
            if (k == worst_k) {
                unit_t K;
                if (rng_f01() > (unit_t)0.5f) {
                    K = (unit_t)1;
                } else {
                    K = -(unit_t)1;
                }
                nx = kite_kx[k] + K * (leader_x - kite_kx[k]);
                ny = kite_ky[k] + K * (leader_y - kite_ky[k]);
            } else {
                nx = leader_x + coeff * (kite_kx[k] - leader_x);
                ny = leader_y + coeff * (kite_ky[k] - leader_y);
            }
            unit_t ox, oy;
            normalize_pair(nx, ny, leader_x, leader_y, ox, oy);
            kite_kx[k] = ox; kite_ky[k] = oy;
        }

        MIGRATION: for (int k = 0; k < N_KITES; k++) {
            #pragma HLS PIPELINE off
            #pragma HLS unroll factor=1
            #pragma HLS LOOP_TRIPCOUNT min=4 max=4
            int rk;
            if (k == N_KITES-1) {
                rk = 0;
            } else {
                rk = k + 1;
            }

            unit_t cx_ = rng_cauchy_cheap() * (unit_t)0.125f;
            unit_t cy_ = rng_cauchy_cheap() * (unit_t)0.125f;
            unit_t nx, ny;
            if (kite_fit[k] < kite_fit[rk]) {
                nx = kite_kx[k] + cx_ * (kite_kx[k] - levy * leader_x);
                ny = kite_ky[k] + cy_ * (kite_ky[k] - levy * leader_y);
            } else {
                nx = kite_kx[k] + cx_ * (levy * leader_x - m_val * kite_kx[k]);
                ny = kite_ky[k] + cy_ * (levy * leader_y - m_val * kite_ky[k]);
            }
            unit_t ox, oy;
            normalize_pair(nx, ny, leader_x, leader_y, ox, oy);
            kite_kx[k] = ox; kite_ky[k] = oy;
        }

        int  sel_k   = 0;
        fit_t sel_fit = kite_fit[0];
        for (int k = 1; k < N_KITES; k++) {
            #pragma HLS PIPELINE off
            #pragma HLS unroll factor=1
            #pragma HLS LOOP_TRIPCOUNT min=3 max=3
            if (kite_fit[k] < sel_fit) { sel_fit = kite_fit[k]; sel_k = k; }
        }

        unit_t bx = (unit_t)0.5f * kite_kx[sel_k] + (unit_t)0.5f * raw_x;
        unit_t by = (unit_t)0.5f * kite_ky[sel_k] + (unit_t)0.5f * raw_y;
        unit_t tx, ty;
        normalize_pair(bx, by, raw_x, raw_y, tx, ty);
        coord_t bka_step_x = (coord_t)((coord_t)tx * MAX_STEP);
        coord_t bka_step_y = (coord_t)((coord_t)ty * MAX_STEP);

        coord_t min_surf   = (coord_t)999;
        coord_t ccd        = (coord_t)1;
        coord_t cor        = (coord_t)15;
        coord_t cdx        = (coord_t)0;
        coord_t cdy        = (coord_t)0;
        int     cidx       = -1;
        coord_t left_thr   = (coord_t)0;
        coord_t right_thr  = (coord_t)0;

        // ---------------------------------------------------------------------------
        // Threat Assessment
        // ---------------------------------------------------------------------------
        FIND_CLOSEST: for (int i = 0; i < safe_n; i++) {
            #pragma HLS PIPELINE off
            #pragma HLS unroll factor=1
            #pragma HLS LOOP_TRIPCOUNT min=0 max=20
            coord_t odx = curr_x - local_obs[i].x;
            coord_t ody = curr_y - local_obs[i].y;
            coord_t cd  = sqrt_dist(square_distance(odx, ody));
            coord_t r_i = local_obs[i].radius;
            coord_t sf  = cd - r_i;

            if (sf < min_surf) {
                min_surf = sf; cidx = i; ccd = cd;
                cor = r_i; cdx = odx; cdy = ody;
            }
            coord_t cross = (coord_t)tx * ody - (coord_t)ty * odx;
            coord_t wt    = r_i / (cd + (coord_t)1);

            if (cross > (coord_t)0.125f) {
                left_thr = left_thr + wt;
            } else if (cross < -(coord_t)0.125f) {
                right_thr = right_thr + wt;
            }
        }

        coord_t des_x  = bka_step_x;
        coord_t des_y  = bka_step_y;
        bool    evade  = false;

        // ---------------------------------------------------------------------------
        // Evasion Logic
        // ---------------------------------------------------------------------------
        if (cidx != -1 && min_surf < LOOKAHEAD_MARGIN) {

            coord_t safe_cd = ccd;
            if (ccd < (coord_t)0.001f) safe_cd = (coord_t)0.001f;

            coord_t inv_cd  = (coord_t)1 / safe_cd;
            coord_t rep_x   = cdx * inv_cd;
            coord_t rep_y   = cdy * inv_cd;
            coord_t dot     = (coord_t)tx * rep_x + (coord_t)ty * rep_y;

            if (dot > (coord_t)0.65f) {
                in_vortex   = false; frustration = 0;
                des_x = bka_step_x + rep_x * (coord_t)1.5f;
                des_y = bka_step_y + rep_y * (coord_t)1.5f;
            } else {
                evade = true;

                // -----------------------------------------------------------
                // Choose the lower-cost corridor through the known map.
                // Fresh entry can commit immediately; periodic re-votes need
                // a clear advantage so the evasion side does not chatter.
                // -----------------------------------------------------------
                bool entering_fresh   = !in_vortex;
                bool periodic_revote  =  in_vortex && ((t & 7) == 0);

                if (entering_fresh || periodic_revote) {
                    in_vortex = true; frustration = 0;
                    coord_t cw_x = -rep_y, cw_y =  rep_x;
                    coord_t cc_x =  rep_y, cc_y = -rep_x;
                    fit_t cw_cost = corridor_cost(cw_x, cw_y, tx, ty,
                                                  curr_x, curr_y, gx, gy,
                                                  local_obs, safe_n, right_thr);
                    fit_t cc_cost = corridor_cost(cc_x, cc_y, tx, ty,
                                                  curr_x, curr_y, gx, gy,
                                                  local_obs, safe_n, left_thr);
                    fit_t cost_diff;
                    if (cw_cost > cc_cost) cost_diff = cw_cost - cc_cost;
                    else                   cost_diff = cc_cost - cw_cost;

                    if (entering_fresh) {
                        if (cw_cost < cc_cost) {
                            vortex_sign = (unit_t)1;
                        } else if (cc_cost < cw_cost) {
                            vortex_sign = -(unit_t)1;
                        }
                    } else {
                        // periodic_revote: hysteresis -- only flip on a
                        // strong, unambiguous corridor advantage.
                        const fit_t HYST = (fit_t)48;
                        if (cost_diff > HYST) {
                            if (cw_cost < cc_cost) {
                                vortex_sign = (unit_t)1;
                            } else {
                                vortex_sign = -(unit_t)1;
                            }
                        }
                        // else: keep current vortex_sign unchanged
                    }
                }

                if (t >= HIST_SIZE) {
                    int     ci  = (t - HIST_SIZE) & (HIST_SIZE - 1);
                    coord_t hx_ = curr_x - hist_x[ci];
                    coord_t hy_ = curr_y - hist_y[ci];

                    if (square_distance(hx_, hy_) < (dist2_t)2.25f) frustration++;
                    else if (frustration > 0) frustration--;

                    if (frustration > 15) {
                        vortex_sign = -vortex_sign; frustration = -120;
                        unit_t ex_ = -(unit_t)rep_x;
                        unit_t ey_ = -(unit_t)rep_y;
                        kite_kx[0] =  ex_; kite_ky[0] =  ey_;
                        kite_kx[1] = -ey_; kite_ky[1] =  ex_;
                        kite_kx[2] = -ex_; kite_ky[2] = -ey_;
                        kite_kx[3] =  ey_; kite_ky[3] = -ex_;
                    }
                }

                coord_t sc;
                if (min_surf < (coord_t)0) sc = (coord_t)0;
                else if (min_surf > LOOKAHEAD_MARGIN) sc = LOOKAHEAD_MARGIN;
                else sc = min_surf;

                coord_t wv_inner = (coord_t)1 - (sc / LOOKAHEAD_MARGIN);
                if (wv_inner < (coord_t)0) wv_inner = (coord_t)0;
                unit_t wv = (unit_t)sqrt_dist((dist2_t)wv_inner);
                unit_t wb = (unit_t)1 - wv;

                coord_t pt = VISUAL_PADDING + ROBOT_RADIUS + (coord_t)4;
                coord_t pf = (coord_t)0;
                if (min_surf < pt) {
                    pf = (pt - min_surf) * (coord_t)0.5f;
                }

                coord_t tan_x = -rep_y * vortex_sign;
                coord_t tan_y =  rep_x * vortex_sign;
                des_x = bka_step_x * (coord_t)wb + (tan_x * MAX_STEP + rep_x * pf) * (coord_t)wv;
                des_y = bka_step_y * (coord_t)wb + (tan_y * MAX_STEP + rep_y * pf) * (coord_t)wv;
            }
        } else if (cidx != -1) {
            in_vortex = false; frustration = 0;
        }

        // ---------------------------------------------------------------------------
        // Kinematics and Bound Checking
        // ---------------------------------------------------------------------------
        if (curr_x < WALL_RADAR)
            des_x = des_x + (WALL_RADAR - curr_x) * (coord_t)1.5f;
        else if (curr_x > MAP_LIMIT - WALL_RADAR)
            des_x = des_x - (curr_x - (MAP_LIMIT - WALL_RADAR)) * (coord_t)1.5f;
        if (curr_y < WALL_RADAR)
            des_y = des_y + (WALL_RADAR - curr_y) * (coord_t)1.5f;
        else if (curr_y > MAP_LIMIT - WALL_RADAR)
            des_y = des_y - (curr_y - (MAP_LIMIT - WALL_RADAR)) * (coord_t)1.5f;

        coord_t iner;
        if (evade) {
            iner = (coord_t)INERTIA_EVADE;
        } else {
            iner = (coord_t)INERTIA_FREE;
        }

        vel_x = iner * vel_x + ((coord_t)1 - iner) * des_x;
        vel_y = iner * vel_y + ((coord_t)1 - iner) * des_y;

        dist2_t vsq = square_distance(vel_x, vel_y);
        if (vsq > (dist2_t)(MAX_STEP * MAX_STEP)) {
            coord_t safe_vsq = sqrt_dist(vsq);
            if (safe_vsq < (coord_t)0.001f) safe_vsq = (coord_t)0.001f;
            coord_t iv = (coord_t)1 / safe_vsq;
            vel_x = vel_x * iv * MAX_STEP;
            vel_y = vel_y * iv * MAX_STEP;
        }

        curr_x = curr_x + vel_x;
        curr_y = curr_y + vel_y;

        if (cidx != -1) {
            coord_t odx = curr_x - local_obs[cidx].x;
            coord_t ody = curr_y - local_obs[cidx].y;
            coord_t dist = sqrt_dist(square_distance(odx, ody));
            coord_t min_req = local_obs[cidx].radius + ROBOT_RADIUS + (coord_t)0.1f;

            if (dist < min_req) {
                coord_t safe_dist = dist;
                if (dist < (coord_t)0.001f) safe_dist = (coord_t)0.001f;
                coord_t inv_dist = (coord_t)1 / safe_dist;

                curr_x = local_obs[cidx].x + odx * inv_dist * min_req;
                curr_y = local_obs[cidx].y + ody * inv_dist * min_req;

                coord_t nx = odx * inv_dist;
                coord_t ny = ody * inv_dist;
                coord_t v_dot = vel_x * nx + vel_y * ny;
                if (v_dot < (coord_t)0) {
                    vel_x = vel_x - v_dot * nx;
                    vel_y = vel_y - v_dot * ny;
                }
            }
        }

        curr_x = clamp_coord(curr_x);
        curr_y = clamp_coord(curr_y);

        hist_x[hist_slot] = curr_x;
        hist_y[hist_slot] = curr_y;
        hist_slot = (hist_slot + 1) & (HIST_SIZE - 1);

        local_px[t] = (data_t)curr_x;
        local_py[t] = (data_t)curr_y;

#ifndef BKA_NATIVE_FLOAT_CSIM
        rt_raw = (ap_uint<14>)(rt_raw + 10);
        ap_uint<6> nrem = (ap_uint<6>)rt_remainder + 6;
        if (nrem >= 25) { rt_raw = (ap_uint<14>)(rt_raw + 1); nrem = nrem - 25; }
        rt_remainder = nrem.range(4, 0);
#endif
    }

    // ---------------------------------------------------------------------------
    // Write Results
    // ---------------------------------------------------------------------------
    WRITE_X: for (int t = 0; t < actual_iters; t++) {
        #pragma HLS LOOP_FLATTEN off
        #pragma HLS PIPELINE II=1
        path_out_x[t] = local_px[t];
    }
    WRITE_Y: for (int t = 0; t < actual_iters; t++) {
        #pragma HLS LOOP_FLATTEN off
        #pragma HLS PIPELINE II=1
        path_out_y[t] = local_py[t];
    }

    *iterations_used   = actual_iters;
    *goal_reached_flag = reached;
}
