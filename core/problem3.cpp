#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cstring>
#include <algorithm>

#include "../config/params.h"
#include "../config/config.h"
using namespace Config;      // 引入 M_PI, G 等物理常数
const auto& p = Params::p3;  // 指向 Problem 3 的动态参数实例

// --- 1. 基础控制参数映射 ---
const int&    DE_POPULATION_SIZE = p.DE_POPULATION_SIZE;
const int&    DE_MAX_GENERATIONS = p.DE_MAX_GENERATIONS;
const double& DE_F               = p.DE_F;
const double& DE_CR              = p.DE_CR;
const double& DE_TIME_STEP       = p.DE_TIME_STEP;

// --- 2. 数组/边界映射 ---
// 使用指针映射，这样你原本代码里的 DE_MINS[i] 依然有效
const double* DE_MINS = p.DE_MINS;
const double* DE_MAXS = p.DE_MAXS;

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

double calculateScreeningDetails(const double params[8], double intervals[3][2]) {
    double uav_speed = params[0], uav_fly_angle_rad = params[1], t_drop1 = params[2], delay1 = params[3],
           delta_t_drop2 = params[4], delay2 = params[5], delta_t_drop3 = params[6], delay3 = params[7];

    for (int i = 0; i < 3; ++i) { intervals[i][0] = -1.0; intervals[i][1] = -1.0; }

    double t_drop2 = t_drop1 + delta_t_drop2, t_drop3 = t_drop2 + delta_t_drop3;
    double t_det1 = t_drop1 + delay1, t_det2 = t_drop2 + delay2, t_det3 = t_drop3 + delay3;
    double uav_fly_direction[3] = {cos(uav_fly_angle_rad), sin(uav_fly_angle_rad), 0.0};
    double uav_velocity[3]; vec_multiply(uav_velocity, uav_fly_direction, uav_speed);
    double missile_velocity_vec[3]; double temp_vec_1[3];
    vec_subtract(temp_vec_1, FAKE_TARGET_POS, MISSILE_M1_INITIAL_POS);
    vec_normalize(temp_vec_1, temp_vec_1); vec_multiply(missile_velocity_vec, temp_vec_1, MISSILE_VELOCITY);
    double drop_pos1[3], drop_pos2[3], drop_pos3[3];
    vec_multiply(temp_vec_1, uav_velocity, t_drop1); vec_add(drop_pos1, UAV_FY1_INITIAL_POS, temp_vec_1);
    vec_multiply(temp_vec_1, uav_velocity, t_drop2); vec_add(drop_pos2, UAV_FY1_INITIAL_POS, temp_vec_1);
    vec_multiply(temp_vec_1, uav_velocity, t_drop3); vec_add(drop_pos3, UAV_FY1_INITIAL_POS, temp_vec_1);
    double det_pos1[3], det_pos2[3], det_pos3[3];
    double term1[3], term2[3];
    vec_multiply(term1, uav_velocity, delay1); vec_multiply(term2, G, 0.5 * delay1 * delay1); vec_add(temp_vec_1, drop_pos1, term1); vec_add(det_pos1, temp_vec_1, term2);
    vec_multiply(term1, uav_velocity, delay2); vec_multiply(term2, G, 0.5 * delay2 * delay2); vec_add(temp_vec_1, drop_pos2, term1); vec_add(det_pos2, temp_vec_1, term2);
    vec_multiply(term1, uav_velocity, delay3); vec_multiply(term2, G, 0.5 * delay3 * delay3); vec_add(temp_vec_1, drop_pos3, term1); vec_add(det_pos3, temp_vec_1, term2);
    double total_screening_time = 0.0;
    double simulation_start_time = fmin(t_det1, fmin(t_det2, t_det3));
    double simulation_end_time = fmax(t_det1, fmax(t_det2, t_det3)) + SMOKE_EFFECTIVE_DURATION;
    for (double t = simulation_start_time; t < simulation_end_time; t += DE_TIME_STEP) {
        double current_missile_pos[3];
        vec_multiply(temp_vec_1, missile_velocity_vec, t);
        vec_add(current_missile_pos, MISSILE_M1_INITIAL_POS, temp_vec_1);
        int s1 = 0, s2 = 0, s3 = 0;
        if (t >= t_det1 && t < t_det1 + SMOKE_EFFECTIVE_DURATION) {
            double cloud1_center[3]; double sink_time = t - t_det1;
            vec_multiply(temp_vec_1, SMOKE_CLOUD_SINK_VELOCITY, sink_time);
            vec_add(cloud1_center, det_pos1, temp_vec_1);
            if (is_screened_by_cloud(current_missile_pos, cloud1_center)) {
                s1 = 1; if (intervals[0][0] < 0) intervals[0][0] = t; intervals[0][1] = t;
            }
        }
        if (t >= t_det2 && t < t_det2 + SMOKE_EFFECTIVE_DURATION) {
            double cloud2_center[3]; double sink_time = t - t_det2;
            vec_multiply(temp_vec_1, SMOKE_CLOUD_SINK_VELOCITY, sink_time);
            vec_add(cloud2_center, det_pos2, temp_vec_1);
            if (is_screened_by_cloud(current_missile_pos, cloud2_center)) {
                s2 = 1; if (intervals[1][0] < 0) intervals[1][0] = t; intervals[1][1] = t;
            }
        }
        if (t >= t_det3 && t < t_det3 + SMOKE_EFFECTIVE_DURATION) {
            double cloud3_center[3]; double sink_time = t - t_det3;
            vec_multiply(temp_vec_1, SMOKE_CLOUD_SINK_VELOCITY, sink_time);
            vec_add(cloud3_center, det_pos3, temp_vec_1);
            if (is_screened_by_cloud(current_missile_pos, cloud3_center)) {
                s3 = 1; if (intervals[2][0] < 0) intervals[2][0] = t; intervals[2][1] = t;
            }
        }
        if (s1 || s2 || s3) total_screening_time += DE_TIME_STEP;
    }
    return total_screening_time;
}

double random_double(double min, double max) {
    return min + ((double)rand() / RAND_MAX) * (max - min);
}

void solve_problem_3() {
    srand(time(NULL));
    const double* mins = DE_MINS;
    const double* maxs = DE_MAXS;

    double population[DE_POPULATION_SIZE][8];
    double fitness[DE_POPULATION_SIZE];
    double gbest_position[8];
    double gbest_fitness = -1.0;
    double gbest_intervals[3][2];

    for (int i = 0; i < DE_POPULATION_SIZE; i++) {
        for (int j = 0; j < 8; j++) {
            population[i][j] = random_double(mins[j], maxs[j]);
        }
        double current_intervals[3][2];
        fitness[i] = calculateScreeningDetails(population[i], current_intervals);

        if (fitness[i] > gbest_fitness) {
            gbest_fitness = fitness[i];
            memcpy(gbest_position, population[i], 8 * sizeof(double));
            memcpy(gbest_intervals, current_intervals, 3 * 2 * sizeof(double));
        }
    }

    printf("--- 开始差分进化 (共 %d 代) ---\n", DE_MAX_GENERATIONS);
    printf(">>> [初始种群] 发现最优解! 时间: %.4fs | C1:[%.2f-%.2f] C2:[%.2f-%.2f] C3:[%.2f-%.2f]\n",
           gbest_fitness, gbest_intervals[0][0], gbest_intervals[0][1], gbest_intervals[1][0], gbest_intervals[1][1], gbest_intervals[2][0], gbest_intervals[2][1]);

    for (int gen = 0; gen < DE_MAX_GENERATIONS; gen++) {
        for (int i = 0; i < DE_POPULATION_SIZE; i++) {
            int a, b, c;
            do { a = rand() % DE_POPULATION_SIZE; } while (a == i);
            do { b = rand() % DE_POPULATION_SIZE; } while (b == i || b == a);
            do { c = rand() % DE_POPULATION_SIZE; } while (c == i || c == a || c == b);

            double mutant_vector[8];
            for (int j = 0; j < 8; j++) {
                mutant_vector[j] = population[a][j] + DE_F * (population[b][j] - population[c][j]);
            }

            double trial_vector[8];
            int rand_j = rand() % 8;
            for (int j = 0; j < 8; j++) {
                if (random_double(0, 1) < DE_CR || j == rand_j) {
                    trial_vector[j] = mutant_vector[j];
                } else {
                    trial_vector[j] = population[i][j];
                }
                if (trial_vector[j] < mins[j]) trial_vector[j] = mins[j];
                if (trial_vector[j] > maxs[j]) trial_vector[j] = maxs[j];
            }

            double current_intervals[3][2];
            double trial_fitness = calculateScreeningDetails(trial_vector, current_intervals);

            if (trial_fitness > fitness[i]) {
                fitness[i] = trial_fitness;
                memcpy(population[i], trial_vector, 8 * sizeof(double));
                if (trial_fitness > gbest_fitness) {
                    gbest_fitness = trial_fitness;
                    memcpy(gbest_position, trial_vector, 8 * sizeof(double));
                    memcpy(gbest_intervals, current_intervals, 3 * 2 * sizeof(double));
                     printf(">>> [第 %3d 代] 发现全局最优解! 时间: %.4fs | C1:[%.2f-%.2f] C2:[%.2f-%.2f] C3:[%.2f-%.2f]\n",
                           gen + 1, gbest_fitness, gbest_intervals[0][0], gbest_intervals[0][1], gbest_intervals[1][0], gbest_intervals[1][1], gbest_intervals[2][0], gbest_intervals[2][1]);
                }
            }
        }
        if ((gen + 1) % 10 == 0) {
            printf("--- [第 %3d / %d 代] 当前最优时间: %.4f s ---\n", gen + 1, DE_MAX_GENERATIONS, gbest_fitness);
        }
    }

    printf("\n--- 优化完成：最优策略如下 ---\n");
    printf("无人机飞行速度: %.4f m/s\n", gbest_position[0]);
    printf("无人机飞行方向: %.4f rad (%.2f 度)\n", gbest_position[1], gbest_position[1] * 180.0 / M_PI);
    printf("首弹投放时间:   %.4f s\n", gbest_position[2]);
    printf("首弹起爆延迟:   %.4f s\n", gbest_position[3]);
    printf("二弹投放间隔:   %.4f s\n", gbest_position[4]);
    printf("二弹起爆延迟:   %.4f s\n", gbest_position[5]);
    printf("三弹投放间隔:   %.4f s\n", gbest_position[6]);
    printf("三弹起爆延迟:   %.4f s\n", gbest_position[7]);
    printf("\n在此策略下，可获得的最大有效遮蔽时间为: %.4f s\n", gbest_fitness);
}

int main() {
    solve_problem_3();
    return 0;
}