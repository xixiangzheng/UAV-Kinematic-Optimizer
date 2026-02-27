#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cfloat>
#include <cstring>
#include <filesystem>

#include "../config/params.h"
#include "../config/config.h"
using namespace Config;      // 引入物理常数
const auto& p = Params::p5;  // 指向 Problem 5 的参数实例

// --- 1. 网格搜索与迭代控制 ---
const int&    DROP_STEPS     = p.DROP_STEPS;
const int&    DELAY_STEPS    = p.DELAY_STEPS;
const int&    MAX_ITERATIONS = p.MAX_ITERATIONS;
const double& TOLERANCE      = p.TOLERANCE;
const double& STEP_TIME_STEP = p.STEP_TIME_STEP;

// --- 2. 核心步长参数 (Hill Climbing Step Sizes) ---
const double& STEP_SIZE_V     = p.STEP_SIZE_V;
const double& STEP_SIZE_THETA = p.STEP_SIZE_THETA;
const double& STEP_SIZE_TDROP = p.STEP_SIZE_TDROP;
const double& STEP_SIZE_DELAY = p.STEP_SIZE_DELAY;

// --- 3. 自适应控制与边界 ---
const double& REDUCE_FACTOR       = p.REDUCE_FACTOR;
const int&    MAX_STEP_REDUCTION  = p.MAX_STEP_REDUCTION;
const double& MIN_DROP            = p.MIN_DROP;
const double& MAX_DROP            = p.MAX_DROP;
const double& MIN_DELAY           = p.MIN_DELAY;
const double& MAX_DELAY           = p.MAX_DELAY;

const int SMOKE_STEPS = (int) (SMOKE_EFFECTIVE_DURATION / STEP_TIME_STEP);

// 名称映射（输出用）
const char* UAV_NAMES[5] = {"FY1", "FY2", "FY3", "FY4", "FY5"};
const char* MISSILE_NAMES[3] = {"M1", "M2", "M3"};

// 向量运算函数（中文注释）
void vec_subtract(double* result, const double* a, const double* b) {
    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
    result[2] = a[2] - b[2];
}

void vec_add(double* result, const double* a, const double* b) {
    result[0] = a[0] + b[0];
    result[1] = a[1] + b[1];
    result[2] = a[2] + b[2];
}

void vec_multiply(double* result, const double* a, double scalar) {
    result[0] = a[0] * scalar;
    result[1] = a[1] * scalar;
    result[2] = a[2] * scalar;
}

double vec_magnitude(const double* a) {
    return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

void vec_normalize(double* result, const double* a) {
    double mag = vec_magnitude(a);
    if (mag > 1e-8) {
        vec_multiply(result, a, 1.0 / mag);
    } else {
        result[0] = result[1] = result[2] = 0.0;
    }
}

// 计算点到线段的距离
double calculate_distance_to_segment(const double* point, const double* seg_start, const double* seg_end, double* t) {
    double seg_vec[3], point_to_start_vec[3];
    vec_subtract(seg_vec, seg_end, seg_start);
    vec_subtract(point_to_start_vec, point, seg_start);

    double dot_product = point_to_start_vec[0] * seg_vec[0] + 
                         point_to_start_vec[1] * seg_vec[1] + 
                         point_to_start_vec[2] * seg_vec[2];
    double seg_len_sq = seg_vec[0] * seg_vec[0] + seg_vec[1] * seg_vec[1] + seg_vec[2] * seg_vec[2];

    *t = (seg_len_sq < 1e-8) ? 0.0 : dot_product / seg_len_sq;
    *t = (*t < 0.0) ? 0.0 : (*t > 1.0) ? 1.0 : *t;

    double closest_point[3];
    vec_multiply(closest_point, seg_vec, *t);
    vec_add(closest_point, seg_start, closest_point);

    double dist_vec[3];
    vec_subtract(dist_vec, point, closest_point);
    return vec_magnitude(dist_vec);
}

// 提取遮蔽区间
void extract_screening_intervals(const char cover_state[], double t_boom, 
                                double intervals[][2], int* interval_cnt) {
    *interval_cnt = 0;
    int in_cover = 0;
    double start_t = 0.0;

    for (int step = 0; step < SMOKE_STEPS; step++) {
        double current_t = t_boom + step * STEP_TIME_STEP;

        if (cover_state[step] == 1 && !in_cover) {
            start_t = current_t;
            in_cover = 1;
        } else if (cover_state[step] == 0 && in_cover) {
            if (*interval_cnt >= MAX_INTERVALS) {
                printf("  Warning: Too many intervals, truncating\n");
                in_cover = 0;
                continue;
            }
            intervals[*interval_cnt][0] = start_t;
            intervals[*interval_cnt][1] = current_t - STEP_TIME_STEP;
            (*interval_cnt)++;
            in_cover = 0;
        }
    }

    if (in_cover && *interval_cnt < MAX_INTERVALS) {
        intervals[*interval_cnt][0] = start_t;
        intervals[*interval_cnt][1] = t_boom + (SMOKE_STEPS - 1) * STEP_TIME_STEP;
        (*interval_cnt)++;
    }
}

// 计算单枚烟雾弹对单个导弹的遮蔽时间+状态
double calculate_grenade_missile_screening(int uav_idx, double uav_v, double uav_theta,
                                          double t_drop, double delay, int missile_idx,
                                          char cover_state[]) {
    double total_time = 0.0;
    double theta_rad = uav_theta * M_PI / 180.0;
    memset(cover_state, 0, SMOKE_STEPS * sizeof(char));

    // 无人机速度向量
    double uav_dir[3] = {cos(theta_rad), sin(theta_rad), 0.0};
    double uav_velocity[3];
    vec_multiply(uav_velocity, uav_dir, uav_v);

    // 投放位置
    double drop_pos[3], temp_vec[3];
    vec_multiply(temp_vec, uav_velocity, t_drop);
    vec_add(drop_pos, UAV_INITIAL_POS[uav_idx], temp_vec);

    // 爆炸位置与时间
    double detonate_time = t_drop + delay;
    double detonate_pos[3];
    vec_multiply(temp_vec, uav_velocity, delay);
    vec_add(detonate_pos, drop_pos, temp_vec);
    detonate_pos[2] -= 0.5 *(-G[2]) * delay * delay;
    detonate_pos[2] = (detonate_pos[2] < 0) ? 0 : detonate_pos[2];

    // 导弹速度向量（直指假目标）
    double missile_dir[3], MISSILE_VELOCITYelocity[3];
    vec_subtract(missile_dir, FAKE_TARGET_POS, MISSILE_INITIAL_POS[missile_idx]);
    vec_normalize(missile_dir, missile_dir);
    vec_multiply(MISSILE_VELOCITYelocity, missile_dir, MISSILE_VELOCITY);

    // 仿真遮蔽状态
    for (int step = 0; step < SMOKE_STEPS; step++) {
        double current_t = detonate_time + step * STEP_TIME_STEP;

        // 烟雾位置
        double smoke_pos[3];
        double sink_time = current_t - detonate_time;
        vec_multiply(temp_vec, (const double[]){0, 0, -SMOKE_CLOUD_SINK_VELOCITY[2]}, sink_time);
        vec_add(smoke_pos, detonate_pos, temp_vec);
        smoke_pos[2] = (smoke_pos[2] < 0) ? 0 : smoke_pos[2];

        // 导弹位置
        double missile_pos[3];
        vec_multiply(temp_vec, MISSILE_VELOCITYelocity, current_t);
        vec_add(missile_pos, MISSILE_INITIAL_POS[missile_idx], temp_vec);
        if (vec_magnitude(missile_pos) < 1e-8) break;

        // 真目标4角点
        double horizontal_dir[3] = {
            TRUE_TARGET_POS_BOTTOM_CENTER[0] - missile_pos[0],
            TRUE_TARGET_POS_BOTTOM_CENTER[1] - missile_pos[1],
            0.0
        };
        double perp_dir[3] = {-horizontal_dir[1], horizontal_dir[0], 0.0};
        vec_normalize(perp_dir, perp_dir);
        double radius_vec[3];
        vec_multiply(radius_vec, perp_dir, TARGET_RADIUS);

        double target_bl[3], target_br[3], target_tl[3], target_tr[3];
        vec_subtract(target_bl, TRUE_TARGET_POS_BOTTOM_CENTER, radius_vec);
        vec_add(target_br, TRUE_TARGET_POS_BOTTOM_CENTER, radius_vec);
        vec_subtract(target_tl, TRUE_TARGET_POS_TOP_CENTER, radius_vec);
        vec_add(target_tr, TRUE_TARGET_POS_TOP_CENTER, radius_vec);

        // 遮蔽判断
        double t_bl, t_br, t_tl, t_tr;
        double dist_bl = calculate_distance_to_segment(smoke_pos, missile_pos, target_bl, &t_bl);
        double dist_br = calculate_distance_to_segment(smoke_pos, missile_pos, target_br, &t_br);
        double dist_tl = calculate_distance_to_segment(smoke_pos, missile_pos, target_tl, &t_tl);
        double dist_tr = calculate_distance_to_segment(smoke_pos, missile_pos, target_tr, &t_tr);

        int is_covered = (dist_bl <= SMOKE_CLOUD_RADIUS - 1e-8 && t_bl >= 0.0 && t_bl <= 1.0) &&
                        (dist_br <= SMOKE_CLOUD_RADIUS - 1e-8 && t_br >= 0.0 && t_br <= 1.0) &&
                        (dist_tl <= SMOKE_CLOUD_RADIUS - 1e-8 && t_tl >= 0.0 && t_tl <= 1.0) &&
                        (dist_tr <= SMOKE_CLOUD_RADIUS - 1e-8 && t_tr >= 0.0 && t_tr <= 1.0);

        if (is_covered) {
            total_time += STEP_TIME_STEP;
            cover_state[step] = 1;
        }
    }

    return total_time;
}

// 打印更优解信息（格式统一）
void print_better_solution(int uav_input_num, int grenade_idx, int missile_idx,
                          double t_drop, double delay, double total_screening,
                          double intervals[][2], int interval_cnt) {
    double detonate_time = t_drop + delay;
    printf("\n[Better Solution Found] %s Grenade %d -> %s\n",
           UAV_NAMES[uav_input_num - 1], grenade_idx + 1, MISSILE_NAMES[missile_idx]);
    printf("  Drop Time: %.2f s\n", t_drop);
    printf("  Detonation Delay: %.2f s\n", delay);
    printf("  Detonation Time: %.2f s\n", detonate_time);
    printf("  Total Screening Time: %.2f s\n", total_screening);
    printf("  Screening Intervals (start-end, s): ");
    if (interval_cnt == 0) {
        printf("No screening\n");
    } else {
        for (int i = 0; i < interval_cnt; i++) {
            printf("[%.2f, %.2f] ", intervals[i][0], intervals[i][1]);
        }
        printf("\n");
    }
    printf("--------------------------------------------------\n");
}

// 优化单枚烟雾弹对单个导弹（目标：最大化该导弹的遮蔽时间）
void optimize_grenade_single_missile(int uav_input_num, int grenade_idx, int missile_idx,
                                    double uav_v, double uav_theta,
                                    double* best_t_drop, double* best_delay, double* best_total) {
    // 初始化最佳参数（初始最佳时间设为-1，确保首次解能被识别）
    *best_total = -1.0;
    *best_t_drop = 0.0;
    *best_delay = 0.1;

    int total_steps = DROP_STEPS * DELAY_STEPS;
    int current_step = 0;

    // 打印优化开始信息
    printf("\n=== Optimizing %s Grenade %d (Target: %s) ===\n",
           UAV_NAMES[uav_input_num - 1], grenade_idx + 1, MISSILE_NAMES[missile_idx]);
    printf("Search Range: Drop Time [%.1f, %.1f]s, Delay [%.1f, %.1f]s\n",
           MIN_DROP, MAX_DROP, MIN_DELAY, MAX_DELAY);
    printf("Progress: ");

    // 网格搜索
    for (int i = 0; i < DROP_STEPS; i++) {
        double t_drop = MIN_DROP + (MAX_DROP - MIN_DROP) * i / (DROP_STEPS - 1);

        for (int j = 0; j < DELAY_STEPS; j++) {
            current_step++;
            double delay = MIN_DELAY + (MAX_DELAY - MIN_DELAY) * j / (DELAY_STEPS - 1);
            char cover_state[SMOKE_STEPS];
            double current_total = calculate_grenade_missile_screening(
                uav_input_num - 1, uav_v, uav_theta, t_drop, delay, missile_idx, cover_state
            );

            // 进度条更新（1%-100%）
            int progress = (int)((double)current_step / total_steps * 100);
            progress = (progress < 1) ? 1 : (progress > 100) ? 100 : progress;
            printf("\rProgress: [");
            for (int k = 0; k < 50; k++) {
                printf("%c", (k < progress / 2) ? '=' : (k == progress / 2) ? '>' : ' ');
            }
            printf("] %3d%%", progress);
            fflush(stdout);

            // 找到更优解，打印信息
            if (current_total > *best_total) {
                *best_total = current_total;
                *best_t_drop = t_drop;
                *best_delay = delay;

                // 提取遮蔽区间并打印
                double intervals[MAX_INTERVALS][2];
                int interval_cnt = 0;
                double detonate_time = t_drop + delay;
                extract_screening_intervals(cover_state, detonate_time, intervals, &interval_cnt);
                print_better_solution(uav_input_num, grenade_idx, missile_idx,
                                    t_drop, delay, current_total, intervals, interval_cnt);
            }
        }
    }

    // 进度条最终状态（100%）
    printf("\rProgress: [==================================================>] 100%%\n");
    fflush(stdout);
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


/**
 * 自动化 CSV 加载函数
 * CSV 格式约定: 
 * 名称, 弹1目标, 弹2目标, 弹3目标, v, rad, (后续参数可有可无)
 * 示例: FY2, M2, M1, M3, 60.0, 1.04, ...
 */
void get_user_input(int* uav_input_num, double* uav_v, double* uav_theta, int targets[GRENADE_COUNT]) {
    const std::string filename = "problem5-grid-search-strategy.csv";
    
    // 1. 获取自动探测后的文件路径
    fs::path target_path = get_csv_path(filename);
    if (target_path.empty()) {
        printf("错误：在 ./data/ 或其父目录中未找到 %s\n", filename.c_str());
        exit(1);
    }

    // 2. 打开文件
    FILE* fp = fopen(target_path.string().c_str(), "r");
    if (!fp) {
        printf("错误：无法打开文件 %s\n", target_path.string().c_str());
        exit(1);
    }

    char line[512];
    bool data_found = false;

    // 3. 读取第一行符合格式的数据
    while (fgets(line, sizeof(line), fp)) {
        char name[16], m1_str[8], m2_str[8], m3_str[8];
        double temp_v, temp_rad;
        
        // 解析格式: 名称, 目标1, 目标2, 目标3, v, rad, (后续忽略)
        // 注意: %[^,] 表示读取到逗号为止
        int fields = sscanf(line, "%[^,],%[^,],%[^,],%[^,],%lf,%lf", 
                            name, m1_str, m2_str, m3_str, &temp_v, &temp_rad);

        if (fields >= 6) {
            int id = 0;
            if (sscanf(name, "FY%d", &id) == 1) {
                if (id < 2 || id > 5) continue; // 仅处理 FY2-FY5

                *uav_input_num = id;
                *uav_v = temp_v;
                *uav_theta = temp_rad * (180.0 / M_PI);

                // 4. 将 "Mx" 字符串转换为整数索引
                char* m_strs[3] = {m1_str, m2_str, m3_str};
                for (int i = 0; i < 3; i++) {
                    int m_id = 0;
                    if (sscanf(m_strs[i], "M%d", &m_id) == 1) {
                        targets[i] = m_id - 1; // M1 -> 0, M2 -> 1, M3 -> 2
                    } else {
                        printf("警告：无法解析目标映射 %s，默认设为 M1\n", m_strs[i]);
                        targets[i] = 0;
                    }
                }

                data_found = true;
                break; 
            }
        }
    }
    fclose(fp);

    if (!data_found) {
        printf("错误：在 %s 中未找到有效的无人机数据（需满足 FYx, Mz, My, Mx... 格式）。\n", filename.c_str());
        exit(1);
    }

    // 5. 最终确认输出
    printf("\n>>> AUTO-LOADED CONFIGURATION <<<\n");
    printf("UAV Type   : %s (ID: %d)\n", UAV_NAMES[*uav_input_num - 1], *uav_input_num);
    printf("UAV Speed  : %.2f m/s\n", *uav_v);
    printf("UAV Angle  : %.2f degrees\n", *uav_theta);
    printf("Target Map : 1->%s, 2->%s, 3->%s\n", 
           MISSILE_NAMES[targets[0]], MISSILE_NAMES[targets[1]], MISSILE_NAMES[targets[2]]);
    printf("----------------------------------\n\n");
}

void solve_problem_5_grid_search(){
    srand((unsigned int)time(NULL));
    int uav_input_num, targets[GRENADE_COUNT];
    double uav_v, uav_theta;

    // 1. 获取用户输入
    get_user_input(&uav_input_num, &uav_v, &uav_theta, targets);

    // 2. 获取当前无人机的烟雾弹目标导弹列表
    const int* grenade_targets = targets;
    if (!grenade_targets) {
        printf("Error: No target mapping found!\n");
        return;
    }

    // 3. 依次优化3枚烟雾弹（每枚对应指定导弹）
    double best_t_drop[GRENADE_COUNT], best_delay[GRENADE_COUNT], best_total[GRENADE_COUNT];
    for (int g = 0; g < GRENADE_COUNT; g++) {
        int target_missile = grenade_targets[g];
        optimize_grenade_single_missile(uav_input_num, g, target_missile,
                                      uav_v, uav_theta,
                                      &best_t_drop[g], &best_delay[g], &best_total[g]);

        // 打印该烟雾弹的最终最佳结果
        char cover_state[SMOKE_STEPS];
        calculate_grenade_missile_screening(uav_input_num - 1, uav_v, uav_theta,
                                          best_t_drop[g], best_delay[g], target_missile, cover_state);
        double intervals[MAX_INTERVALS][2];
        int interval_cnt = 0;
        double detonate_time = best_t_drop[g] + best_delay[g];
        extract_screening_intervals(cover_state, detonate_time, intervals, &interval_cnt);

        printf("\n=== Final Best Result for %s Grenade %d -> %s ===\n",
               UAV_NAMES[uav_input_num - 1], g + 1, MISSILE_NAMES[target_missile]);
        printf("  Drop Time: %.2f s\n", best_t_drop[g]);
        printf("  Detonation Delay: %.2f s\n", best_delay[g]);
        printf("  Detonation Time: %.2f s\n", detonate_time);
        printf("  Total Screening Time: %.2f s\n", best_total[g]);
        printf("  Screening Intervals (start-end, s): ");
        if (interval_cnt == 0) {
            printf("No screening\n");
        } else {
            for (int i = 0; i < interval_cnt; i++) {
                printf("[%.2f, %.2f] ", intervals[i][0], intervals[i][1]);
            }
            printf("\n");
        }
        printf("==================================================\n\n");
    }
}

int main() {
    solve_problem_5_grid_search();
    return 0;
}