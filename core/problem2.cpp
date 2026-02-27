#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cstring>
#include <algorithm>

#include "../config/params.h"
#include "../config/config.h"
using namespace Config; // 允许直接访问物理常量 (G, MISSILE_VELOCITY等)
const auto& p = Params::p2; // 获取动态参数容器

// --- 算法控制参数映射 (Shadowing) ---
const double& OPT_TIME_STEP   = p.OPT_TIME_STEP;
const double& OPT_SPEED_MIN   = p.OPT_SPEED_MIN;
const double& OPT_SPEED_MAX   = p.OPT_SPEED_MAX;
const double& OPT_T_DROP_MIN  = p.OPT_T_DROP_MIN;
const double& OPT_T_DROP_MAX  = p.OPT_T_DROP_MAX;
const double& OPT_DELAY_MIN   = p.OPT_DELAY_MIN;
const double& OPT_DELAY_MAX   = p.OPT_DELAY_MAX;

const int&    GLOBAL_SEARCH_ITERATIONS   = p.GLOBAL_SEARCH_ITERATIONS;
const int&    LOCAL_SEARCH_ROUNDS        = p.LOCAL_SEARCH_ROUNDS;
const int&    ITERATIONS_PER_ROUND       = p.ITERATIONS_PER_ROUND;

const double& LOCAL_SEARCH_INITIAL_RANGE = p.LOCAL_SEARCH_INITIAL_RANGE;
const double& LOCAL_SEARCH_DECAY_RATE    = p.LOCAL_SEARCH_DECAY_RATE;


void vec_add(double res[3], const double v1[3], const double v2[3]) {
    res[0] = v1[0] + v2[0];
    res[1] = v1[1] + v2[1];
    res[2] = v1[2] + v2[2];
}

void vec_subtract(double res[3], const double v1[3], const double v2[3]) {
    res[0] = v1[0] - v2[0];
    res[1] = v1[1] - v2[1];
    res[2] = v1[2] - v2[2];
}

void vec_multiply(double res[3], const double v[3], double scalar) {
    res[0] = v[0] * scalar;
    res[1] = v[1] * scalar;
    res[2] = v[2] * scalar;
}

double vec_dot(const double v1[3], const double v2[3]) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

double vec_norm(const double v[3]) {
    return sqrt(vec_dot(v, v));
}

void vec_normalize(double res[3], const double v[3]) {
    double norm = vec_norm(v);
    if (norm > 1e-9) {
        res[0] = v[0] / norm;
        res[1] = v[1] / norm;
        res[2] = v[2] / norm;
    } else {
        res[0] = 0;
        res[1] = 0;
        res[2] = 0;
    }
}

double calculateDistancePointToSegment(const double p[3], const double a[3], const double b[3], double* t_out) {
    double ab[3], ap[3];
    vec_subtract(ab, b, a);
    vec_subtract(ap, p, a);
    double ab_norm_sq = vec_dot(ab, ab);
    if (ab_norm_sq < 1e-9) {
        *t_out = 0.0;
        return vec_norm(ap);
    }
    double t = vec_dot(ap, ab) / ab_norm_sq;
    *t_out = t;
    double projection[3];
    if (t < 0.0) {
        projection[0] = a[0]; projection[1] = a[1]; projection[2] = a[2];
    } else if (t > 1.0) {
        projection[0] = b[0]; projection[1] = b[1]; projection[2] = b[2];
    } else {
        double ab_t[3];
        vec_multiply(ab_t, ab, t);
        vec_add(projection, a, ab_t);
    }
    double p_minus_proj[3];
    vec_subtract(p_minus_proj, p, projection);
    return vec_norm(p_minus_proj);
}

int is_screened_by_cloud(const double missile_pos[3], const double cloud_center[3]) {
    double diff_vec[3];
    vec_subtract(diff_vec, missile_pos, cloud_center);
    if (vec_norm(diff_vec) <= SMOKE_CLOUD_RADIUS) {
        return 1;
    }
    double horizontal_dir_to_cyl[3] = {TRUE_TARGET_POS_BOTTOM_CENTER[0] - missile_pos[0], TRUE_TARGET_POS_BOTTOM_CENTER[1] - missile_pos[1], 0.0};
    double perp_vec[3] = {-horizontal_dir_to_cyl[1], horizontal_dir_to_cyl[0], 0.0};
    double perp_unit_vec[3];
    vec_normalize(perp_unit_vec, perp_vec);
    double radius_vec[3];
    vec_multiply(radius_vec, perp_unit_vec, TARGET_RADIUS);
    double target_bl[3], target_br[3], target_tl[3], target_tr[3];
    vec_subtract(target_bl, TRUE_TARGET_POS_BOTTOM_CENTER, radius_vec);
    vec_add(target_br, TRUE_TARGET_POS_BOTTOM_CENTER, radius_vec);
    vec_subtract(target_tl, TRUE_TARGET_POS_TOP_CENTER, radius_vec);
    vec_add(target_tr, TRUE_TARGET_POS_TOP_CENTER, radius_vec);
    double t_param;
    double dist_bl = calculateDistancePointToSegment(cloud_center, missile_pos, target_bl, &t_param);
    if (dist_bl > SMOKE_CLOUD_RADIUS || t_param < 0.0 || t_param > 1.0) return 0;
    double dist_br = calculateDistancePointToSegment(cloud_center, missile_pos, target_br, &t_param);
    if (dist_br > SMOKE_CLOUD_RADIUS || t_param < 0.0 || t_param > 1.0) return 0;
    double dist_tl = calculateDistancePointToSegment(cloud_center, missile_pos, target_tl, &t_param);
    if (dist_tl > SMOKE_CLOUD_RADIUS || t_param < 0.0 || t_param > 1.0) return 0;
    double dist_tr = calculateDistancePointToSegment(cloud_center, missile_pos, target_tr, &t_param);
    if (dist_tr > SMOKE_CLOUD_RADIUS || t_param < 0.0 || t_param > 1.0) return 0;
    return 1;
}

double calculateScreeningTime(double uav_speed, double uav_fly_angle_rad, double t_drop, double detonation_delay) {
    const double t_det = t_drop + detonation_delay;
    double uav_fly_direction[3] = {cos(uav_fly_angle_rad), sin(uav_fly_angle_rad), 0.0};
    double uav_velocity[3];
    vec_multiply(uav_velocity, uav_fly_direction, uav_speed);
    double missile_velocity_vec[3];
    double temp_vec_1[3];
    vec_subtract(temp_vec_1, FAKE_TARGET_POS, MISSILE_M1_INITIAL_POS);
    vec_normalize(temp_vec_1, temp_vec_1);
    vec_multiply(missile_velocity_vec, temp_vec_1, MISSILE_VELOCITY);
    double drop_pos[3];
    vec_multiply(temp_vec_1, uav_velocity, t_drop);
    vec_add(drop_pos, UAV_FY1_INITIAL_POS, temp_vec_1);
    double grenade_fly_time = detonation_delay;
    double term1[3], term2[3], detonation_pos[3];
    vec_multiply(term1, uav_velocity, grenade_fly_time);
    vec_multiply(term2, G, 0.5 * grenade_fly_time * grenade_fly_time);
    vec_add(temp_vec_1, drop_pos, term1);
    vec_add(detonation_pos, temp_vec_1, term2);
    double total_screening_time = 0.0;
    const double simulation_start_time = t_det;
    const double simulation_end_time = t_det + SMOKE_EFFECTIVE_DURATION;
    for (double t = simulation_start_time; t < simulation_end_time; t += OPT_TIME_STEP) {
        double current_missile_pos[3], current_smoke_pos[3];
        vec_multiply(temp_vec_1, missile_velocity_vec, t);
        vec_add(current_missile_pos, MISSILE_M1_INITIAL_POS, temp_vec_1);
        double cloud_sink_time = t - t_det;
        vec_multiply(temp_vec_1, SMOKE_CLOUD_SINK_VELOCITY, cloud_sink_time);
        vec_add(current_smoke_pos, detonation_pos, temp_vec_1);
        if (is_screened_by_cloud(current_missile_pos, current_smoke_pos)) {
            total_screening_time += OPT_TIME_STEP;
        }
    }
    return total_screening_time;
}

double random_double(double min, double max) {
    return min + ((double)rand() / RAND_MAX) * (max - min);
}

void solve_problem_2() {
    srand(time(NULL));
    const double ANGLE_MIN = 0.0, ANGLE_MAX = 2.0 * M_PI;
    double best_time = -1.0;
    double best_params[4] = {0};
    printf("--- 阶段一：全局探索 (共 %d 次迭代) ---\n", GLOBAL_SEARCH_ITERATIONS);
    for (int i = 0; i < GLOBAL_SEARCH_ITERATIONS; ++i) {
        double speed = random_double(OPT_SPEED_MIN, OPT_SPEED_MAX);
        double angle = random_double(ANGLE_MIN, ANGLE_MAX);
        double t_drop = random_double(OPT_T_DROP_MIN, OPT_T_DROP_MAX);
        double delay = random_double(OPT_DELAY_MIN, OPT_DELAY_MAX);
        double current_time = calculateScreeningTime(speed, angle, t_drop, delay);
        if (current_time > best_time) {
            best_time = current_time;
            best_params[0] = speed;
            best_params[1] = angle;
            best_params[2] = t_drop;
            best_params[3] = delay;
            printf(">>> [全局迭代 %5d] 新最优解! 时间: %.4fs (速度:%.2f, 角度:%.2f, 投放:%.2f, 延迟:%.2f)\n",
                   i + 1, best_time, speed, angle, t_drop, delay);
        }
        if ((i + 1) % 100 == 0) {
            printf("--- [全局进度 %5d / %d] 当前最优时间: %.4f s ---\n", i + 1, GLOBAL_SEARCH_ITERATIONS, best_time);
        }
    }
    double speed_range = (OPT_SPEED_MAX - OPT_SPEED_MIN) * LOCAL_SEARCH_INITIAL_RANGE;
    double angle_range = (ANGLE_MAX - ANGLE_MIN) * LOCAL_SEARCH_INITIAL_RANGE;
    double t_drop_range = (OPT_T_DROP_MAX - OPT_T_DROP_MIN) * LOCAL_SEARCH_INITIAL_RANGE;
    double delay_range = (OPT_DELAY_MAX - OPT_DELAY_MIN) * LOCAL_SEARCH_INITIAL_RANGE;
    for (int r = 0; r < LOCAL_SEARCH_ROUNDS; ++r) {
        printf("\n--- 阶段二：局部寻优 - 第 %d / %d 轮 (范围缩减至 %.1f%%) ---\n", r + 1, LOCAL_SEARCH_ROUNDS, (speed_range / ((OPT_SPEED_MAX - OPT_SPEED_MIN) * LOCAL_SEARCH_INITIAL_RANGE)) * 100.0);
        for (int i = 0; i < ITERATIONS_PER_ROUND; ++i) {
            double speed = random_double(std::max(OPT_SPEED_MIN, best_params[0] - speed_range / 2.0), std::min(OPT_SPEED_MAX, best_params[0] + speed_range / 2.0));
            double angle = random_double(std::max(ANGLE_MIN, best_params[1] - angle_range / 2.0), std::min(ANGLE_MAX, best_params[1] + angle_range / 2.0));
            double t_drop = random_double(std::max(OPT_T_DROP_MIN, best_params[2] - t_drop_range / 2.0), std::min(OPT_T_DROP_MAX, best_params[2] + t_drop_range / 2.0));
            double delay = random_double(std::max(OPT_DELAY_MIN, best_params[3] - delay_range / 2.0), std::min(OPT_DELAY_MAX, best_params[3] + delay_range / 2.0));
            double current_time = calculateScreeningTime(speed, angle, t_drop, delay);
            if (current_time > best_time) {
                best_time = current_time;
                best_params[0] = speed;
                best_params[1] = angle;
                best_params[2] = t_drop;
                best_params[3] = delay;
                printf(">>> [局部迭代 %d-%d] 新最优解! 时间: %.4fs (速度:%.2f, 角度:%.2f, 投放:%.2f, 延迟:%.2f)\n",
                       r + 1, i + 1, best_time, speed, angle, t_drop, delay);
            }
            if ((i + 1) % 100 == 0) {
                 printf("--- [局部进度 %d-%d / %d] 当前最优时间: %.4f s ---\n", r + 1, i + 1, ITERATIONS_PER_ROUND, best_time);
            }
        }
        speed_range *= LOCAL_SEARCH_DECAY_RATE;
        angle_range *= LOCAL_SEARCH_DECAY_RATE;
        t_drop_range *= LOCAL_SEARCH_DECAY_RATE;
        delay_range *= LOCAL_SEARCH_DECAY_RATE;
    }
    printf("\n--- 优化完成：最优策略如下 ---\n");
    printf("无人机飞行速度 (v):      %.4f m/s\n", best_params[0]);
    printf("无人机飞行方向 (theta):  %.4f rad (%.2f 度)\n", best_params[1], best_params[1] * 180.0 / M_PI);
    printf("烟幕弹投放时间 (t_drop): %.4f s\n", best_params[2]);
    printf("烟幕弹起爆延迟 (delay):  %.4f s\n", best_params[3]);
    double best_speed = best_params[0];
    double best_angle = best_params[1];
    double best_t_drop = best_params[2];
    double best_delay = best_params[3];
    double uav_fly_direction[3] = {cos(best_angle), sin(best_angle), 0.0};
    double uav_velocity[3];
    vec_multiply(uav_velocity, uav_fly_direction, best_speed);
    double drop_pos[3];
    double temp_vec[3];
    vec_multiply(temp_vec, uav_velocity, best_t_drop);
    vec_add(drop_pos, UAV_FY1_INITIAL_POS, temp_vec);
    double grenade_fly_time_final = best_delay;
    double term1_final[3], term2_final[3], detonation_pos_final[3];
    vec_multiply(term1_final, uav_velocity, grenade_fly_time_final);
    vec_multiply(term2_final, G, 0.5 * grenade_fly_time_final * grenade_fly_time_final);
    vec_add(temp_vec, drop_pos, term1_final);
    vec_add(detonation_pos_final, temp_vec, term2_final);
    printf("\n计算出的投放点坐标: (%.2f, %.2f, %.2f)\n", drop_pos[0], drop_pos[1], drop_pos[2]);
    printf("计算出的起爆点坐标: (%.2f, %.2f, %.2f)\n", detonation_pos_final[0], detonation_pos_final[1], detonation_pos_final[2]);
    printf("\n在此策略下，可获得的最大有效遮蔽时间为: %.4f s\n", best_time);
}

int main() {
    solve_problem_2();
    return 0;
}