#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cstring>
#include <algorithm>

#include "../config/config.h"
using namespace Config;

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

void solve_problem_1() {
    const double uav_speed = DEFAULT_UAV_SPEED;
    const double t_drop = DEFAULT_T_DROP;
    const double detonation_delay = DEFAULT_DETONATION_DELAY;
    const double t_det = t_drop + detonation_delay;

    double uav_fly_direction[3], uav_velocity[3];
    double temp_vec_1[3];

    double uav_target_horizontal[3] = {FAKE_TARGET_POS[0], FAKE_TARGET_POS[1], UAV_FY1_INITIAL_POS[2]};
    vec_subtract(temp_vec_1, uav_target_horizontal, UAV_FY1_INITIAL_POS);
    vec_normalize(uav_fly_direction, temp_vec_1);
    vec_multiply(uav_velocity, uav_fly_direction, uav_speed);

    double MISSILE_VELOCITYelocity_vec[3];
    vec_subtract(temp_vec_1, FAKE_TARGET_POS, MISSILE_M1_INITIAL_POS);
    vec_normalize(temp_vec_1, temp_vec_1);
    vec_multiply(MISSILE_VELOCITYelocity_vec, temp_vec_1, MISSILE_VELOCITY);

    double drop_pos[3];
    vec_multiply(temp_vec_1, uav_velocity, t_drop);
    vec_add(drop_pos, UAV_FY1_INITIAL_POS, temp_vec_1);

    double grenade_fly_time = detonation_delay;
    double term1[3], term2[3], detonation_pos[3];
    vec_multiply(term1, uav_velocity, grenade_fly_time);
    vec_multiply(term2, G, 0.5 * grenade_fly_time * grenade_fly_time);
    vec_add(temp_vec_1, drop_pos, term1);
    vec_add(detonation_pos, temp_vec_1, term2);

    printf("--- 详细仿真过程 ---\n");
    printf("| Time(s) | Missile Coords (x,y,z)        | UAV Coords (x,y,z)            | Smoke Coords (x,y,z)          | Max LOS Dist(m) | Screened?   |\n");

    double total_screening_time = 0.0;
    const long long print_step_interval = (long long)round(SIM_PRINT_INTERVAL / SIM_TIME_STEP);
    long long step_counter = 0;

    const double simulation_start_time = t_det;
    const double simulation_end_time = t_det + SMOKE_EFFECTIVE_DURATION;

    for (double t = 0; t <= simulation_end_time; t += SIM_TIME_STEP) {
        double current_missile_pos[3], current_uav_pos[3], current_smoke_pos[3];
        vec_multiply(temp_vec_1, MISSILE_VELOCITYelocity_vec, t);
        vec_add(current_missile_pos, MISSILE_M1_INITIAL_POS, temp_vec_1);

        vec_multiply(temp_vec_1, uav_velocity, t);
        vec_add(current_uav_pos, UAV_FY1_INITIAL_POS, temp_vec_1);

        if (t < t_drop) {
            current_smoke_pos[0] = current_uav_pos[0]; current_smoke_pos[1] = current_uav_pos[1]; current_smoke_pos[2] = current_uav_pos[2];
        } else if (t < t_det) {
            double flight_time = t - t_drop;
            vec_multiply(term1, uav_velocity, flight_time);
            vec_multiply(term2, G, 0.5 * flight_time * flight_time);
            vec_add(temp_vec_1, drop_pos, term1);
            vec_add(current_smoke_pos, temp_vec_1, term2);
        } else {
            double cloud_sink_time = t - t_det;
            vec_multiply(temp_vec_1, SMOKE_CLOUD_SINK_VELOCITY, cloud_sink_time);
            vec_add(current_smoke_pos, detonation_pos, temp_vec_1);
        }

        double distance_to_show = -1.0;
        int screened_status = 0;

        if (t >= simulation_start_time) {
            screened_status = is_screened_by_cloud(current_missile_pos, current_smoke_pos);
            if (screened_status) {
                total_screening_time += SIM_TIME_STEP;
            }

            double horizontal_dir_to_cyl[3] = {TRUE_TARGET_POS_BOTTOM_CENTER[0] - current_missile_pos[0], TRUE_TARGET_POS_BOTTOM_CENTER[1] - current_missile_pos[1], 0.0};
            double perp_vec[3] = {-horizontal_dir_to_cyl[1], horizontal_dir_to_cyl[0], 0.0};
            double perp_unit_vec[3]; vec_normalize(perp_unit_vec, perp_vec);
            double radius_vec[3]; vec_multiply(radius_vec, perp_unit_vec, TARGET_RADIUS);
            double target_bl[3], target_br[3], target_tl[3], target_tr[3];
            vec_subtract(target_bl, TRUE_TARGET_POS_BOTTOM_CENTER, radius_vec);
            vec_add(target_br, TRUE_TARGET_POS_BOTTOM_CENTER, radius_vec);
            vec_subtract(target_tl, TRUE_TARGET_POS_TOP_CENTER, radius_vec);
            vec_add(target_tr, TRUE_TARGET_POS_TOP_CENTER, radius_vec);
            double t_param;
            double dist_bl = calculateDistancePointToSegment(current_smoke_pos, current_missile_pos, target_bl, &t_param);
            double dist_br = calculateDistancePointToSegment(current_smoke_pos, current_missile_pos, target_br, &t_param);
            double dist_tl = calculateDistancePointToSegment(current_smoke_pos, current_missile_pos, target_tl, &t_param);
            double dist_tr = calculateDistancePointToSegment(current_smoke_pos, current_missile_pos, target_tr, &t_param);
            distance_to_show = std::max({dist_bl, dist_br, dist_tl, dist_tr});
        }

        if (step_counter % print_step_interval == 0) {
            char missile_str[100], uav_str[100], smoke_str[100];
            sprintf(missile_str, "(%.2f, %.2f, %.2f)", current_missile_pos[0], current_missile_pos[1], current_missile_pos[2]);
            sprintf(uav_str, "(%.2f, %.2f, %.2f)", current_uav_pos[0], current_uav_pos[1], current_uav_pos[2]);
            sprintf(smoke_str, "(%.2f, %.2f, %.2f)", current_smoke_pos[0], current_smoke_pos[1], current_smoke_pos[2]);

            printf("| %-7.2f | %-29s | %-29s | %-29s |", t, missile_str, uav_str, smoke_str);

            if (distance_to_show >= 0) {
                printf(" %-15.2f | %-11s |\n", distance_to_show, (screened_status ? "Yes" : "No"));
            } else {
                printf(" %-15s | %-11s |\n", "N/A", "No");
            }
        }
        step_counter++;
    }

    printf("\n--- 最终计算结果 ---\n");
    printf("烟幕干扰弹对M1的有效遮蔽总时长为: %.4f 秒\n", total_screening_time);
}

int main() {
    solve_problem_1();
    return 0;
}