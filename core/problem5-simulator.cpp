#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cfloat>
#include <cstring>
#include <filesystem>

#include "../config/config.h"
using namespace Config;

void vec_add(double res[3], const double v1[3], const double v2[3]) { res[0] = v1[0] + v2[0]; res[1] = v1[1] + v2[1]; res[2] = v1[2] + v2[2]; }
void vec_subtract(double res[3], const double v1[3], const double v2[3]) { res[0] = v1[0] - v2[0]; res[1] = v1[1] - v2[1]; res[2] = v1[2] - v2[2]; }
void vec_multiply(double res[3], const double v[3], double scalar) { res[0] = v[0] * scalar; res[1] = v[1] * scalar; res[2] = v[2] * scalar; }
double vec_dot(const double v1[3], const double v2[3]) { return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]; }
double vec_norm(const double v[3]) { return sqrt(vec_dot(v, v)); }
void vec_normalize(double res[3], const double v[3]) { double norm = vec_norm(v); if (norm > 1e-9) { res[0] = v[0] / norm; res[1] = v[1] / norm; res[2] = v[2] / norm; } else { res[0] = 0; res[1] = 0; res[2] = 0; } }

double MISSILE_VELOCITY_VEC[3][3];

double calculateDistancePointToSegment(const double p[3], const double a[3], const double b[3], double* t_out) {
    double ab[3], ap[3]; vec_subtract(ab, b, a); vec_subtract(ap, p, a);
    double ab_norm_sq = vec_dot(ab, ab);
    if (ab_norm_sq < 1e-9) { *t_out = 0.0; return vec_norm(ap); }
    double t = vec_dot(ap, ab) / ab_norm_sq; *t_out = t;
    double projection[3];
    if (t < 0.0) { projection[0] = a[0]; projection[1] = a[1]; projection[2] = a[2]; }
    else if (t > 1.0) { projection[0] = b[0]; projection[1] = b[1]; projection[2] = b[2]; }
    else { double ab_t[3]; vec_multiply(ab_t, ab, t); vec_add(projection, a, ab_t); }
    double p_minus_proj[3]; vec_subtract(p_minus_proj, p, projection);
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
    double perp_unit_vec[3]; vec_normalize(perp_unit_vec, perp_vec);
    double radius_vec[3]; vec_multiply(radius_vec, perp_unit_vec, TARGET_RADIUS);
    double target_bl[3], target_br[3], target_tl[3], target_tr[3];
    vec_subtract(target_bl, TRUE_TARGET_POS_BOTTOM_CENTER, radius_vec); vec_add(target_br, TRUE_TARGET_POS_BOTTOM_CENTER, radius_vec);
    vec_subtract(target_tl, TRUE_TARGET_POS_TOP_CENTER, radius_vec); vec_add(target_tr, TRUE_TARGET_POS_TOP_CENTER, radius_vec);
    double t_bl, t_br, t_tl, t_tr;

    double dist_bl = calculateDistancePointToSegment(cloud_center, missile_pos, target_bl, &t_bl);
    if (dist_bl > SMOKE_CLOUD_RADIUS || t_bl < 0.0 || t_bl > 1.0) return 0;

    double dist_br = calculateDistancePointToSegment(cloud_center, missile_pos, target_br, &t_br);
    if (dist_br > SMOKE_CLOUD_RADIUS || t_br < 0.0 || t_br > 1.0) return 0;

    double dist_tl = calculateDistancePointToSegment(cloud_center, missile_pos, target_tl, &t_tl);
    if (dist_tl > SMOKE_CLOUD_RADIUS || t_tl < 0.0 || t_tl > 1.0) return 0;

    double dist_tr = calculateDistancePointToSegment(cloud_center, missile_pos, target_tr, &t_tr);
    if (dist_tr > SMOKE_CLOUD_RADIUS || t_tr < 0.0 || t_tr > 1.0) return 0;

    return 1;
}

double calculate_union_screening_time(int num_uavs, const int uav_indices[], const double params[][8], int num_missiles, const int missile_indices[]) {
    int total_bombs = num_uavs * 3;
    if (total_bombs == 0) return 0.0;

    double det_times[total_bombs];
    double det_pos[total_bombs][3];

    double min_start_t = 1e9, max_end_t = -1.0;
    int bomb_idx = 0;

    for (int i = 0; i < num_uavs; ++i) {
        int uav_idx = uav_indices[i];
        double uav_speed = params[i][0], uav_fly_angle_rad = params[i][1];
        double t_drop1 = params[i][2], delay1 = params[i][3];
        double delta_t_drop2 = params[i][4], delay2 = params[i][5];
        double delta_t_drop3 = params[i][6], delay3 = params[i][7];

        double uav_fly_direction[3] = {cos(uav_fly_angle_rad), sin(uav_fly_angle_rad), 0.0};
        double uav_velocity[3]; vec_multiply(uav_velocity, uav_fly_direction, uav_speed);

        double drop_times[3] = {t_drop1, t_drop1 + delta_t_drop2, t_drop1 + delta_t_drop2 + delta_t_drop3};
        double delays[3] = {delay1, delay2, delay3};

        for (int j = 0; j < 3; ++j) {
            det_times[bomb_idx] = drop_times[j] + delays[j];
            min_start_t = fmin(min_start_t, det_times[bomb_idx]);
            max_end_t = fmax(max_end_t, det_times[bomb_idx] + SMOKE_EFFECTIVE_DURATION);

            double drop_pos[3], temp_vec[3], term1[3], term2[3];
            vec_multiply(temp_vec, uav_velocity, drop_times[j]);
            vec_add(drop_pos, UAV_INITIAL_POS[uav_idx], temp_vec);
            vec_multiply(term1, uav_velocity, delays[j]);
            vec_multiply(term2, G, 0.5 * delays[j] * delays[j]);
            vec_add(temp_vec, drop_pos, term1);
            vec_add(det_pos[bomb_idx], temp_vec, term2);
            bomb_idx++;
        }
    }

    if (max_end_t < 0) return 0.0;
    double total_time_union = 0.0;
    for (double t = min_start_t; t < max_end_t; t += SIM_TIME_STEP) {
        int is_any_missile_screened_this_step = 0;
        for (int m = 0; m < num_missiles; ++m) {
            int missile_idx = missile_indices[m];
            double missile_pos[3], temp_vec[3];
            vec_multiply(temp_vec, MISSILE_VELOCITY_VEC[missile_idx], t);
            vec_add(missile_pos, MISSILE_INITIAL_POS[missile_idx], temp_vec);

            int is_missile_m_blocked = 0;
            for (int b = 0; b < total_bombs; ++b) {
                if (t >= det_times[b] && t < det_times[b] + SMOKE_EFFECTIVE_DURATION) {
                    double cloud_center[3];
                    vec_multiply(temp_vec, SMOKE_CLOUD_SINK_VELOCITY, t - det_times[b]);
                    vec_add(cloud_center, det_pos[b], temp_vec);
                    if (is_screened_by_cloud(missile_pos, cloud_center)) {
                        is_missile_m_blocked = 1;
                        break;
                    }
                }
            }
            if(is_missile_m_blocked){
                is_any_missile_screened_this_step = 1;
                break;
            }
        }
        if(is_any_missile_screened_this_step){
            total_time_union += SIM_TIME_STEP;
        }
    }
    return total_time_union;
}

double calculate_interval_union_duration(const double intervals[3][2]) {
    double sorted_intervals[3][2];
    memcpy(sorted_intervals, intervals, 3 * 2 * sizeof(double));

    int valid_count = 0;
    for (int i = 0; i < 3; i++) {
        if (sorted_intervals[i][0] >= 0) {
            if (valid_count != i) {
                sorted_intervals[valid_count][0] = sorted_intervals[i][0];
                sorted_intervals[valid_count][1] = sorted_intervals[i][1];
            }
            valid_count++;
        }
    }
    if (valid_count == 0) return 0.0;

    for (int i = 0; i < valid_count - 1; i++) {
        for (int j = 0; j < valid_count - i - 1; j++) {
            if (sorted_intervals[j][0] > sorted_intervals[j+1][0]) {
                double temp_start = sorted_intervals[j][0];
                double temp_end = sorted_intervals[j][1];
                sorted_intervals[j][0] = sorted_intervals[j+1][0];
                sorted_intervals[j][1] = sorted_intervals[j+1][1];
                sorted_intervals[j+1][0] = temp_start;
                sorted_intervals[j+1][1] = temp_end;
            }
        }
    }

    double total_duration = 0;
    double current_start = sorted_intervals[0][0];
    double current_end = sorted_intervals[0][1];

    for (int i = 1; i < valid_count; i++) {
        if (sorted_intervals[i][0] <= current_end + 1e-9) {
            current_end = fmax(current_end, sorted_intervals[i][1]);
        } else {
            total_duration += (current_end - current_start);
            current_start = sorted_intervals[i][0];
            current_end = sorted_intervals[i][1];
        }
    }
    total_duration += (current_end - current_start);
    return total_duration;
}


void generate_output_file_p5(const double final_params[][8], const double results[5][3][3][2]) {
    printf("\n--- 结果文件 (result3.xlsx) CSV格式输出 ---\n");
    printf("请将以下内容复制到CSV文件中:\n\n");
    printf("无人机编号,无人机运动方向,无人机运动速度 (m/s),烟幕干扰弹编号,烟幕干扰弹投放点的x坐标 (m),烟幕干扰弹投放点的y坐标 (m),烟幕干扰弹投放点的z坐标 (m),烟幕干扰弹起爆点的x坐标 (m),烟幕干扰弹起爆点的y坐标 (m),烟幕干扰弹起爆点的z坐标 (m),有效干扰时长 (s),干扰的导弹编号\n");

    char uav_names[5][4] = {"FY1", "FY2", "FY3", "FY4", "FY5"};

    for (int i=0; i<5; ++i) {
        double uav_speed = final_params[i][0];
        double uav_fly_angle_rad = final_params[i][1];

        double uav_fly_direction[3] = {cos(uav_fly_angle_rad), sin(uav_fly_angle_rad), 0.0};
        double uav_velocity[3];
        vec_multiply(uav_velocity, uav_fly_direction, uav_speed);

        double angle_deg = uav_fly_angle_rad * 180.0 / M_PI;
        if (angle_deg < 0) angle_deg += 360.0;

        double t_drop1 = final_params[i][2];
        double delta_t_drop2 = final_params[i][4];
        double delta_t_drop3 = final_params[i][6];
        double drop_times[3] = {t_drop1, t_drop1 + delta_t_drop2, t_drop1 + delta_t_drop2 + delta_t_drop3};
        double delays[3] = {final_params[i][3], final_params[i][5], final_params[i][7]};

        for (int j=0; j<3; ++j) {
            double drop_pos[3];
            double temp_vec_1[3];
            vec_multiply(temp_vec_1, uav_velocity, drop_times[j]);
            vec_add(drop_pos, UAV_INITIAL_POS[i], temp_vec_1);

            double det_pos[3];
            double term1[3], term2[3];
            vec_multiply(term1, uav_velocity, delays[j]);
            vec_multiply(term2, G, 0.5 * delays[j] * delays[j]);
            vec_add(temp_vec_1, drop_pos, term1);
            vec_add(det_pos, temp_vec_1, term2);

            double individual_duration = calculate_interval_union_duration(results[i][j]);
            char missile_str[20] = "";
            int first = 1;
            for (int k=0; k<3; ++k) {
                if (results[i][j][k][0] >= 0) {
                    if (!first) strcat(missile_str, ",");
                    char temp[4];
                    sprintf(temp, "M%d", k+1);
                    strcat(missile_str, temp);
                    first = 0;
                }
            }
            if (strlen(missile_str) == 0) strcpy(missile_str, "无");


            if (j==0){
                 printf("%s,%.4f,%.4f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%s\n",
                        uav_names[i], angle_deg, uav_speed, j+1,
                        drop_pos[0], drop_pos[1], drop_pos[2],
                        det_pos[0], det_pos[1], det_pos[2],
                        individual_duration, missile_str);
            } else {
                 printf(",,,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%s\n",
                        j+1,
                        drop_pos[0], drop_pos[1], drop_pos[2],
                        det_pos[0], det_pos[1], det_pos[2],
                        individual_duration, missile_str);
            }
        }
    }
}


void run_detailed_simulation_and_print_results(const double final_params[][8]) {
    double results[5][3][3][2];
    for(int i=0; i<5; ++i) for(int j=0; j<3; ++j) for(int k=0; k<3; ++k) {
        results[i][j][k][0] = -1.0; results[i][j][k][1] = -1.0;
    }

    double det_times[15];
    double det_pos[15][3];
    int uav_owner_of_bomb[15];

    double min_start_t = 1e9, max_end_t = -1.0;
    int bomb_idx = 0;

    for (int i = 0; i < 5; ++i) {
        double uav_speed = final_params[i][0], uav_fly_angle_rad = final_params[i][1];
        double t_drop1 = final_params[i][2], delay1 = final_params[i][3];
        double delta_t_drop2 = final_params[i][4], delay2 = final_params[i][5];
        double delta_t_drop3 = final_params[i][6], delay3 = final_params[i][7];

        double uav_fly_direction[3] = {cos(uav_fly_angle_rad), sin(uav_fly_angle_rad), 0.0};
        double uav_velocity[3]; vec_multiply(uav_velocity, uav_fly_direction, uav_speed);

        double drop_times[3] = {t_drop1, t_drop1 + delta_t_drop2, t_drop1 + delta_t_drop2 + delta_t_drop3};
        double delays[3] = {delay1, delay2, delay3};

        for (int j = 0; j < 3; ++j) {
            uav_owner_of_bomb[bomb_idx] = i;
            det_times[bomb_idx] = drop_times[j] + delays[j];
            min_start_t = fmin(min_start_t, det_times[bomb_idx]);
            max_end_t = fmax(max_end_t, det_times[bomb_idx] + SMOKE_EFFECTIVE_DURATION);

            double drop_pos[3], temp_vec[3], term1[3], term2[3];
            vec_multiply(temp_vec, uav_velocity, drop_times[j]);
            vec_add(drop_pos, UAV_INITIAL_POS[i], temp_vec);
            vec_multiply(term1, uav_velocity, delays[j]);
            vec_multiply(term2, G, 0.5 * delays[j] * delays[j]);
            vec_add(temp_vec, drop_pos, term1);
            vec_add(det_pos[bomb_idx], temp_vec, term2);
            bomb_idx++;
        }
    }

    if (max_end_t < 0) return;
    for (double t = 0; t < max_end_t; t += SIM_TIME_STEP) {
        for (int m = 0; m < 3; ++m) {
            double missile_pos[3], temp_vec[3];
            vec_multiply(temp_vec, MISSILE_VELOCITY_VEC[m], t);
            vec_add(missile_pos, MISSILE_INITIAL_POS[m], temp_vec);

            for (int b = 0; b < 15; ++b) {
                if (t >= det_times[b] && t < det_times[b] + SMOKE_EFFECTIVE_DURATION) {
                    double cloud_center[3];
                    vec_multiply(temp_vec, SMOKE_CLOUD_SINK_VELOCITY, t - det_times[b]);
                    vec_add(cloud_center, det_pos[b], temp_vec);
                    if (is_screened_by_cloud(missile_pos, cloud_center)) {
                        int uav_idx = uav_owner_of_bomb[b];
                        int bomb_num = b % 3;
                        if (results[uav_idx][bomb_num][m][0] < 0.0) {
                            results[uav_idx][bomb_num][m][0] = t;
                        }
                        results[uav_idx][bomb_num][m][1] = t;
                    }
                }
            }
        }
    }

    printf("\n--- 详细仿真结果：各云团对各导弹的有效遮蔽区间 ---\n");
    char uav_names[5][4] = {"FY1", "FY2", "FY3", "FY4", "FY5"};
    for (int i = 0; i < 5; ++i) {
        printf("--- 无人机 %s ---\n", uav_names[i]);
        for (int j = 0; j < 3; ++j) {
            printf("  - 烟幕弹 %d:\n", j + 1);
            for (int k = 0; k < 3; ++k) {
                if (results[i][j][k][0] >= 0) {
                    printf("    - 对 M%d 遮蔽区间: [%.2f s, %.2f s]\n", k + 1, results[i][j][k][0], results[i][j][k][1]);
                } else {
                    printf("    - 对 M%d 遮蔽区间: [无有效遮蔽]\n", k + 1);
                }
            }
        }
    }

    printf("\n--- 最终评估：计算总任务贡献时长 ---\n");

    double total_sum_of_times = 0;
    int all_uavs[] = {0,1,2,3,4};
    int m1_indices[] = {0}, m2_indices[] = {1}, m3_indices[] = {2};

    printf("正在计算对 M1 的总遮蔽时长...\n");
    double time_vs_m1 = calculate_union_screening_time(5, all_uavs, final_params, 1, m1_indices);
    printf("...对 M1 的总遮蔽时长 (并集) 为: %.4f s\n", time_vs_m1);

    printf("正在计算对 M2 的总遮蔽时长...\n");
    double time_vs_m2 = calculate_union_screening_time(5, all_uavs, final_params, 1, m2_indices);
    printf("...对 M2 的总遮蔽时长 (并集) 为: %.4f s\n", time_vs_m2);

    printf("正在计算对 M3 的总遮蔽时长...\n");
    double time_vs_m3 = calculate_union_screening_time(5, all_uavs, final_params, 1, m3_indices);
    printf("...对 M3 的总遮蔽时长 (并集) 为: %.4f s\n", time_vs_m3);

    total_sum_of_times = time_vs_m1 + time_vs_m2 + time_vs_m3;
    printf("最终评估完成！\n");

    printf("\n--- 最终策略总结 ---\n");
    printf("在此策略下，可获得的最大有效遮蔽时间 (各导弹遮蔽时长总和) 为: %.4f s\n", total_sum_of_times);

    generate_output_file_p5(final_params, results);
}

namespace fs = std::filesystem;
fs::path get_csv_path(const std::string& filename) {
    // 自动探测路径：优先查找 ./data/，找不到则查找 ../data/
    fs::path pathA = fs::current_path()  / "data" / filename;
    if (fs::exists(pathA)) {
        return pathA;
    }

    fs::path pathB = fs::current_path() / ".." / "data" / filename;
    if (fs::exists(pathB)) {
        return pathB;
    }

    fs::path pathC = fs::current_path()  / ".." / ".." / "data" / filename;
    if (fs::exists(pathC)) {
        return pathC;
    }

    // 如果都找不到，返回空路径
    return fs::path();
}

// 从 CSV 读取参数的工具函数
int load_params_from_csv(const char* filename, double params[5][8]) {
    FILE* fp = fopen(filename, "r");
    if (!fp) {
        printf("错误：无法打开文件 %s\n", filename);
        return 0;
    }

    char line[512];
    int count_loaded = 0;

    while (fgets(line, sizeof(line), fp)) {
        char name[16];
        double temp[8];
        
        // 1. 先把整行读入临时变量
        int fields = sscanf(line, "%[^,],%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                            name, &temp[0], &temp[1], &temp[2], &temp[3],
                            &temp[4], &temp[5], &temp[6], &temp[7]);

        if (fields == 9) {
            // 2. 提取名称中的数字，例如从 "FY3" 中提取 3
            int uav_id = 0;
            if (sscanf(name, "FY%d", &uav_id) == 1) {
                // 3. 计算索引 (FY1 -> index 0, FY2 -> index 1...)
                int index = uav_id - 1;
                
                if (index >= 0 && index < 5) {
                    // 将数据拷贝到正确的位置
                    for (int j = 0; j < 8; j++) {
                        params[index][j] = temp[j];
                    }
                    count_loaded++;
                }
            }
        }
    }

    fclose(fp);
    return count_loaded; // 返回成功匹配并存放的飞机数量
}


/**
 * 基于 final_params[i] 的任务参数映射说明 (包含首列飞机 ID)：
 * ----------------------------------------------------------------------------------
 * 数组索引 | 变量名            | 物理含义                              | 单位
 * ----------------------------------------------------------------------------------
 * [ID列]   | uav_id            | 飞机编号 (1-5, 对应 FY1-FY5)          | -
 * [0]      | uav_speed         | 无人机水平飞行速度                    | m/s
 * [1]      | uav_fly_angle_rad | 无人机飞行航向角                      | rad (弧度)
 * [2]      | t_drop1           | 第 1 枚烟雾弹的绝对投放时间           | s
 * [3]      | delay1            | 第 1 枚烟雾弹从投放到爆炸的延迟时间   | s
 * [4]      | delta_t_drop2     | 第 2 枚与第 1 枚投放的时间间隔 (Δt)   | s
 * [5]      | delay2            | 第 2 枚烟雾弹的引信延迟时间           | s
 * [6]      | delta_t_drop3     | 第 3 枚与第 2 枚投放的时间间隔 (Δt)   | s
 * [7]      | delay3            | 第 3 枚烟雾弹的引信延迟时间           | s
 * ----------------------------------------------------------------------------------
 * * 逻辑转换说明：
 * 1. 存储对应：程序根据 CSV 首列 ID，将数据存入 final_params[ID-1] 数组。
 * 2. 时间计算：第 n 枚(n>1)烟雾弹的绝对投放时间 t_drop_n = t_drop_(n-1) + delta_t_drop_n。
 * 3. 爆炸时间：第 n 枚烟雾弹的绝对爆炸时间 T_exp_n = t_drop_n + delay_n。
 */
 
void solve_problem_5_simulator() {
    srand(time(NULL));

    // --- 1. 智能查找 CSV 文件 ---
    std::string target_file = "problem5-simulator-strategy.csv";
    fs::path csv_path = get_csv_path(target_file);

    if (csv_path.empty()) {
        printf("错误：在 ./data/ 或 ../data/ 下均未找到文件 %s\n", target_file.c_str());
        printf("当前运行目录为: %s\n", fs::current_path().string().c_str());
        return;
    }

    printf("成功找到数据文件: %s\n", csv_path.string().c_str());

    // --- 2. 从 CSV 加载参数 ---
    double final_params[5][8];
    // 使用 .string().c_str() 将 fs::path 转换为 load_params_from_csv 接受的字符串指针
    if (load_params_from_csv(csv_path.string().c_str(), final_params) < 5) {
        printf("警告：加载参数失败，请检查文件内容格式。\n");
        return;
    }

    // --- 3. 后续初始化与仿真逻辑 ---
    printf("--- 阶段一：初始化战场环境 ---\n");
    for (int i=0; i<3; ++i) {
        double temp[3];
        vec_subtract(temp, FAKE_TARGET_POS, MISSILE_INITIAL_POS[i]);
        vec_normalize(MISSILE_VELOCITY_VEC[i], temp);
        vec_multiply(MISSILE_VELOCITY_VEC[i], MISSILE_VELOCITY_VEC[i], MISSILE_VELOCITY);
    }
    printf("导弹速度向量计算完成。\n");
    printf("作战策略已载入。\n\n");

    run_detailed_simulation_and_print_results(final_params);
}

int main() {
    solve_problem_5_simulator();
    return 0;
}