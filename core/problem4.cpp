#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cstring>
#include <algorithm>

#include "../config/params.h"
#include "../config/config.h"
using namespace Config;      // 引入 M_PI, G 等物理常数
const auto& p = Params::p4;  // 指向 Problem 4 的动态参数实例

// --- 1. 协同进化控制参数映射 ---
const int&    CO_POPULATION_SIZE = p.CO_POPULATION_SIZE;
const int&    CO_MAX_GENERATIONS = p.CO_MAX_GENERATIONS;
const double& CO_F               = p.CO_F;
const double& CO_CR              = p.CO_CR;
const double& PARTICIPATION_BONUS = p.PARTICIPATION_BONUS;
const double& CO_TIME_STEP       = p.CO_TIME_STEP;

// --- 2. 搜索边界映射 (指针方式) ---
// [速度, 角度, 投放时间, 延迟]
const double* CO_MINS = p.CO_MINS;
const double* CO_MAXS = p.CO_MAXS;

void vec_add(double res[3], const double v1[3], const double v2[3]) { res[0] = v1[0] + v2[0]; res[1] = v1[1] + v2[1]; res[2] = v1[2] + v2[2]; }
void vec_subtract(double res[3], const double v1[3], const double v2[3]) { res[0] = v1[0] - v2[0]; res[1] = v1[1] - v2[1]; res[2] = v1[2] - v2[2]; }
void vec_multiply(double res[3], const double v[3], double scalar) { res[0] = v[0] * scalar; res[1] = v[1] * scalar; res[2] = v[2] * scalar; }
double vec_dot(const double v1[3], const double v2[3]) { return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]; }
double vec_norm(const double v[3]) { return sqrt(vec_dot(v, v)); }
void vec_normalize(double res[3], const double v[3]) { double norm = vec_norm(v); if (norm > 1e-9) { res[0] = v[0] / norm; res[1] = v[1] / norm; res[2] = v[2] / norm; } else { res[0] = 0; res[1] = 0; res[2] = 0; } }

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

double calculateCombinedScreeningAndIntervals(const double combined_params[12], double intervals[3][1][2]) {
    for (int i=0; i<3; ++i) {intervals[i][0][0] = -1.0; intervals[i][0][1] = -1.0;}

    int num_uavs = 3;
    double det_times[num_uavs];
    double det_pos[num_uavs][3];
    double min_start_t = 1e9, max_end_t = -1.0;

    double missile_velocity_vec[3];
    double temp_vec[3];
    vec_subtract(temp_vec, FAKE_TARGET_POS, MISSILE_M1_INITIAL_POS);
    vec_normalize(temp_vec, temp_vec);
    vec_multiply(missile_velocity_vec, temp_vec, MISSILE_VELOCITY);

    for (int i=0; i<num_uavs; ++i) {
        double uav_speed = combined_params[i*4 + 0];
        double uav_fly_angle_rad = combined_params[i*4 + 1];
        double t_drop = combined_params[i*4 + 2];
        double delay = combined_params[i*4 + 3];
        det_times[i] = t_drop + delay;
        min_start_t = fmin(min_start_t, det_times[i]);
        max_end_t = fmax(max_end_t, det_times[i] + SMOKE_EFFECTIVE_DURATION);

        double uav_fly_direction[3] = {cos(uav_fly_angle_rad), sin(uav_fly_angle_rad), 0.0};
        double uav_velocity[3]; vec_multiply(uav_velocity, uav_fly_direction, uav_speed);
        double drop_pos[3], term1[3], term2[3];
        vec_multiply(temp_vec, uav_velocity, t_drop);
        vec_add(drop_pos, UAV_INITIAL_POS[i], temp_vec);
        vec_multiply(term1, uav_velocity, delay);
        vec_multiply(term2, G, 0.5 * delay * delay);
        vec_add(temp_vec, drop_pos, term1);
        vec_add(det_pos[i], temp_vec, term2);
    }

    double total_time_union = 0.0;
    for (double t = min_start_t; t < max_end_t; t += CO_TIME_STEP) {
        double missile_pos[3];
        vec_multiply(temp_vec, missile_velocity_vec, t);
        vec_add(missile_pos, MISSILE_M1_INITIAL_POS, temp_vec);

        int screened_this_step = 0;
        for (int b = 0; b < num_uavs; ++b) {
            if (t >= det_times[b] && t < det_times[b] + SMOKE_EFFECTIVE_DURATION) {
                double cloud_center[3];
                vec_multiply(temp_vec, SMOKE_CLOUD_SINK_VELOCITY, t - det_times[b]);
                vec_add(cloud_center, det_pos[b], temp_vec);
                if (is_screened_by_cloud(missile_pos, cloud_center)) {
                    screened_this_step = 1;
                    if (intervals[b][0][0] < 0.0) intervals[b][0][0] = t;
                    intervals[b][0][1] = t;
                }
            }
        }
        if(screened_this_step) {
            total_time_union += CO_TIME_STEP;
        }
    }
    return total_time_union;
}

double random_double(double min, double max) { return min + ((double)rand() / RAND_MAX) * (max - min); }

void solve_problem_4() {
    srand(time(NULL));

    const double* mins = CO_MINS;
    const double* maxs = CO_MAXS;

    double population[3][CO_POPULATION_SIZE][4];
    double fitness[3][CO_POPULATION_SIZE];
    double pop_best_params[3][4];
    double pop_best_fitness[3] = {-1.0, -1.0, -1.0};

    double gbest_params[12];
    double gbest_fitness = -1.0;
    double gbest_real_time = -1.0;

    for (int p_idx = 0; p_idx < 3; ++p_idx) {
        for (int i = 0; i < CO_POPULATION_SIZE; ++i) {
            for (int j = 0; j < 4; ++j) {
                population[p_idx][i][j] = random_double(mins[j], maxs[j]);
            }
        }
    }

    printf("--- 开始协同进化 (共 %d 代) ---\n", CO_MAX_GENERATIONS);
    for (int gen = 0; gen < CO_MAX_GENERATIONS; ++gen) {
        for(int p_idx=0; p_idx<3; ++p_idx){
            for(int i=0; i<CO_POPULATION_SIZE; ++i){
                double team[12];
                double intervals[3][1][2];
                for(int j=0; j<4; ++j) team[p_idx*4+j] = population[p_idx][i][j];
                for(int other_p=0; other_p<3; ++other_p){
                    if(p_idx == other_p) continue;
                    if(pop_best_fitness[other_p] < 0){
                         for(int j=0; j<4; ++j) team[other_p*4+j] = population[other_p][rand() % CO_POPULATION_SIZE][j];
                    } else {
                         for(int j=0; j<4; ++j) team[other_p*4+j] = pop_best_params[other_p][j];
                    }
                }
                double real_time = calculateCombinedScreeningAndIntervals(team, intervals);
                int effective_drones = 0;
                if(intervals[0][0][0] >= 0) effective_drones++;
                if(intervals[1][0][0] >= 0) effective_drones++;
                if(intervals[2][0][0] >= 0) effective_drones++;
                fitness[p_idx][i] = real_time + PARTICIPATION_BONUS * effective_drones;

                if(fitness[p_idx][i] > pop_best_fitness[p_idx]){
                    pop_best_fitness[p_idx] = fitness[p_idx][i];
                    for(int j=0; j<4; ++j) pop_best_params[p_idx][j] = population[p_idx][i][j];
                }
            }
        }

        for (int p_idx = 0; p_idx < 3; ++p_idx) {
            for (int i = 0; i < CO_POPULATION_SIZE; ++i) {
                int a, b, c;
                do { a = rand() % CO_POPULATION_SIZE; } while (a == i);
                do { b = rand() % CO_POPULATION_SIZE; } while (b == i || b == a);
                do { c = rand() % CO_POPULATION_SIZE; } while (c == i || c == a || c == b);

                double trial_vector[4];
                int rand_j = rand() % 4;
                for (int j = 0; j < 4; ++j) {
                    double mutant_val = population[p_idx][a][j] + CO_F * (population[p_idx][b][j] - population[p_idx][c][j]);
                    if (random_double(0, 1) < CO_CR || j == rand_j) trial_vector[j] = mutant_val;
                    else trial_vector[j] = population[p_idx][i][j];
                    if (trial_vector[j] < mins[j]) trial_vector[j] = mins[j];
                    if (trial_vector[j] > maxs[j]) trial_vector[j] = maxs[j];
                }

                double team[12];
                double intervals[3][1][2];
                for(int j=0; j<4; ++j) team[p_idx*4+j] = trial_vector[j];
                for(int other_p=0; other_p<3; ++other_p){
                    if(p_idx == other_p) continue;
                    for(int j=0; j<4; ++j) team[other_p*4+j] = pop_best_params[other_p][j];
                }
                double real_time = calculateCombinedScreeningAndIntervals(team, intervals);
                int effective_drones = 0;
                if(intervals[0][0][0] >= 0) effective_drones++;
                if(intervals[1][0][0] >= 0) effective_drones++;
                if(intervals[2][0][0] >= 0) effective_drones++;
                double trial_fitness = real_time + PARTICIPATION_BONUS * effective_drones;

                if (trial_fitness > fitness[p_idx][i]) {
                    fitness[p_idx][i] = trial_fitness;
                    for (int j = 0; j < 4; j++) population[p_idx][i][j] = trial_vector[j];
                }
            }
        }

        double current_best_team[12];
        for(int j=0; j<4; ++j) {
            current_best_team[j] = pop_best_params[0][j];
            current_best_team[j+4] = pop_best_params[1][j];
            current_best_team[j+8] = pop_best_params[2][j];
        }
        double intervals[3][1][2];
        double current_real_time = calculateCombinedScreeningAndIntervals(current_best_team, intervals);
        int current_effective_drones = 0;
        if(intervals[0][0][0] >= 0) current_effective_drones++;
        if(intervals[1][0][0] >= 0) current_effective_drones++;
        if(intervals[2][0][0] >= 0) current_effective_drones++;
        double current_gbest_fitness = current_real_time + PARTICIPATION_BONUS * current_effective_drones;

        if(current_gbest_fitness > gbest_fitness){
            gbest_fitness = current_gbest_fitness;
            gbest_real_time = current_real_time;
            memcpy(gbest_params, current_best_team, 12 * sizeof(double));
            printf(">>> [第 %4d 代] 发现最优协同策略! 真实时间: %.4fs (激励得分 %.2f, %d机协同)\n", gen + 1, gbest_real_time, gbest_fitness, current_effective_drones);
            for (int i=0; i<3; ++i) {
                printf("    FY%d 策略 -> v: %.2f, theta: %.2f, t_drop: %.2f, delay: %.2f | 遮蔽区间: [%.2f-%.2f]\n",
                       i+1, gbest_params[i*4 + 0], gbest_params[i*4 + 1], gbest_params[i*4 + 2], gbest_params[i*4 + 3],
                       intervals[i][0][0], intervals[i][0][1]);
            }
        }

        if ((gen + 1) % 10 == 0) {
            printf("--- [第 %4d / %d 代] 当前最优真实时间: %.4f s ---\n", gen + 1, CO_MAX_GENERATIONS, gbest_real_time);
        }
    }

    printf("\n--- 优化完成：最优策略如下 ---\n");
    char uav_names[3][4] = {"FY1", "FY2", "FY3"};
    for (int i=0; i<3; ++i) {
        printf("--- 无人机 %s ---\n", uav_names[i]);
        printf("飞行速度 (v):      %.4f m/s\n", gbest_params[i*4 + 0]);
        printf("飞行方向 (theta):  %.4f rad (%.2f 度)\n", gbest_params[i*4 + 1], gbest_params[i*4 + 1] * 180.0 / M_PI);
        printf("投放时间 (t_drop): %.4f s\n", gbest_params[i*4 + 2]);
        printf("起爆延迟 (delay):  %.4f s\n", gbest_params[i*4 + 3]);
    }
    printf("\n在此策略下，可获得的最大有效遮蔽时间为: %.4f s\n", gbest_real_time);
}

int main() {
    solve_problem_4();
    return 0;
}