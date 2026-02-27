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

int targets[GRENADE_COUNT];

// 优化参数结构体
typedef struct {
    double uav_v;                     // 无人机速度
    double uav_theta;                 // 无人机角度
    double t_drop[GRENADE_COUNT];     // 投放时间
    double delay[GRENADE_COUNT];      // 延迟时间
    double total_screening_time;      // 总遮蔽时间
} OptimizationParams;

// 名称映射（输出用）
const char* UAV_NAMES[5] = {"FY1", "FY2", "FY3", "FY4", "FY5"};
const char* MISSILE_NAMES[3] = {"M1", "M2", "M3"};

// 向量运算函数
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

// 计算单枚烟雾弹对单个导弹的遮蔽时间
double calculate_grenade_missile_screening(int uav_idx, double uav_v, double uav_theta,
                                          double t_drop, double delay, int missile_idx) {
    double total_time = 0.0;
    double theta_rad = uav_theta * M_PI / 180.0;

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
        }
    }

    return total_time;
}

// 计算总遮蔽时间
double calculate_total_screening_time(int uav_input_num, const OptimizationParams* params) {
    double total = 0.0;
    for (int i = 0; i < GRENADE_COUNT; i++) {
        total += calculate_grenade_missile_screening(
            uav_input_num - 1,
            params->uav_v,
            params->uav_theta,
            params->t_drop[i],
            params->delay[i],
            targets[i]
        );
    }
    return total;
}

// 提取遮蔽区间（用于结果展示）
void extract_screening_intervals(int uav_idx, double uav_v, double uav_theta,
                                double t_drop, double delay, int missile_idx,
                                double t_boom, double intervals[][2], int* interval_cnt) {
    *interval_cnt = 0;
    int in_cover = 0;
    double start_t = 0.0;
    char cover_state[SMOKE_STEPS] = {0};
    double theta_rad = uav_theta * M_PI / 180.0;

    // 无人机速度向量
    double uav_dir[3] = {cos(theta_rad), sin(theta_rad), 0.0};
    double uav_velocity[3];
    vec_multiply(uav_velocity, uav_dir, uav_v);

    // 投放位置
    double drop_pos[3], temp_vec[3];
    vec_multiply(temp_vec, uav_velocity, t_drop);
    vec_add(drop_pos, UAV_INITIAL_POS[uav_idx], temp_vec);

    // 爆炸位置
    double detonate_pos[3];
    vec_multiply(temp_vec, uav_velocity, delay);
    vec_add(detonate_pos, drop_pos, temp_vec);
    detonate_pos[2] -= 0.5 *(-G[2]) * delay * delay;
    detonate_pos[2] = (detonate_pos[2] < 0) ? 0 : detonate_pos[2];

    // 导弹速度向量
    double missile_dir[3], MISSILE_VELOCITYelocity[3];
    vec_subtract(missile_dir, FAKE_TARGET_POS, MISSILE_INITIAL_POS[missile_idx]);
    vec_normalize(missile_dir, missile_dir);
    vec_multiply(MISSILE_VELOCITYelocity, missile_dir, MISSILE_VELOCITY);

    // 计算遮蔽状态
    for (int step = 0; step < SMOKE_STEPS; step++) {
        double current_t = t_boom + step * STEP_TIME_STEP;

        // 烟雾位置
        double smoke_pos[3];
        double sink_time = current_t - t_boom;
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

        cover_state[step] = ((dist_bl <= SMOKE_CLOUD_RADIUS - 1e-8 && t_bl >= 0.0 && t_bl <= 1.0) &&
                           (dist_br <= SMOKE_CLOUD_RADIUS - 1e-8 && t_br >= 0.0 && t_br <= 1.0) &&
                           (dist_tl <= SMOKE_CLOUD_RADIUS - 1e-8 && t_tl >= 0.0 && t_tl <= 1.0) &&
                           (dist_tr <= SMOKE_CLOUD_RADIUS - 1e-8 && t_tr >= 0.0 && t_tr <= 1.0)) ? 1 : 0;
    }

    // 提取区间
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

// 局部爬山法优化
void hill_climbing_optimization(int uav_input_num, OptimizationParams* initial_params) {
    // 初始化当前参数和最佳参数
    OptimizationParams current_params = *initial_params;
    OptimizationParams best_params = *initial_params;

    // 计算初始总遮蔽时间
    best_params.total_screening_time = calculate_total_screening_time(uav_input_num, &best_params);
    current_params.total_screening_time = best_params.total_screening_time;

    printf("\n=== Starting Hill Climbing Optimization ===\n");
    printf("Initial total screening time: %.2f s\n", best_params.total_screening_time);
    printf("Initial parameters:\n");
    printf("  UAV speed: %.2f m/s\n", initial_params->uav_v);
    printf("  UAV angle: %.2f degrees\n", initial_params->uav_theta);
    for (int i = 0; i < GRENADE_COUNT; i++) {
        printf("  Grenade %d: Drop=%.2f s, Delay=%.2f s\n",
               i+1, initial_params->t_drop[i], initial_params->delay[i]);
    }

    double step_v = STEP_SIZE_V;
    double step_theta = STEP_SIZE_THETA;
    double step_tdrop = STEP_SIZE_TDROP;
    double step_delay = STEP_SIZE_DELAY;
    int step_reduction_count = 0;
    int no_improvement_count = 0;

    // 爬山法主循环
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        int improved = 0;
        OptimizationParams new_params = current_params;

        // 尝试优化无人机速度
        new_params.uav_v = current_params.uav_v + step_v;
        if (new_params.uav_v > 0) {  // 速度必须为正
            new_params.total_screening_time = calculate_total_screening_time(uav_input_num, &new_params);
            if (new_params.total_screening_time > best_params.total_screening_time) {
                best_params = new_params;
                improved = 1;
            }
        }

        if (!improved) {
            new_params.uav_v = current_params.uav_v - step_v;
            if (new_params.uav_v > 0) {  // 速度必须为正
                new_params.total_screening_time = calculate_total_screening_time(uav_input_num, &new_params);
                if (new_params.total_screening_time > best_params.total_screening_time) {
                    best_params = new_params;
                    improved = 1;
                }
            }
        }

        // 如果速度优化没有改进，尝试优化角度
        if (!improved) {
            new_params = current_params;
            new_params.uav_theta = fmod(current_params.uav_theta + step_theta, 360.0);
            new_params.total_screening_time = calculate_total_screening_time(uav_input_num, &new_params);
            if (new_params.total_screening_time > best_params.total_screening_time) {
                best_params = new_params;
                improved = 1;
            }
        }

        if (!improved) {
            new_params = current_params;
            new_params.uav_theta = fmod(current_params.uav_theta - step_theta + 360.0, 360.0);
            new_params.total_screening_time = calculate_total_screening_time(uav_input_num, &new_params);
            if (new_params.total_screening_time > best_params.total_screening_time) {
                best_params = new_params;
                improved = 1;
            }
        }

        // 尝试优化每颗烟雾弹的投放时间和延迟时间
        for (int i = 0; i < GRENADE_COUNT && !improved; i++) {
            // 优化投放时间 - 增加
            new_params = current_params;
            new_params.t_drop[i] += step_tdrop;
            if (new_params.t_drop[i] >= 0) {  // 投放时间不能为负
                new_params.total_screening_time = calculate_total_screening_time(uav_input_num, &new_params);
                if (new_params.total_screening_time > best_params.total_screening_time) {
                    best_params = new_params;
                    improved = 1;
                    break;
                }
            }

            // 优化投放时间 - 减少
            if (!improved) {
                new_params = current_params;
                new_params.t_drop[i] -= step_tdrop;
                if (new_params.t_drop[i] >= 0) {  // 投放时间不能为负
                    new_params.total_screening_time = calculate_total_screening_time(uav_input_num, &new_params);
                    if (new_params.total_screening_time > best_params.total_screening_time) {
                        best_params = new_params;
                        improved = 1;
                        break;
                    }
                }
            }

            // 优化延迟时间 - 增加
            if (!improved) {
                new_params = current_params;
                new_params.delay[i] += step_delay;
                if (new_params.delay[i] >= 0.1) {  // 延迟时间不能小于0.1
                    new_params.total_screening_time = calculate_total_screening_time(uav_input_num, &new_params);
                    if (new_params.total_screening_time > best_params.total_screening_time) {
                        best_params = new_params;
                        improved = 1;
                        break;
                    }
                }
            }

            // 优化延迟时间 - 减少
            if (!improved) {
                new_params = current_params;
                new_params.delay[i] -= step_delay;
                if (new_params.delay[i] >= 0.1) {  // 延迟时间不能小于0.1
                    new_params.total_screening_time = calculate_total_screening_time(uav_input_num, &new_params);
                    if (new_params.total_screening_time > best_params.total_screening_time) {
                        best_params = new_params;
                        improved = 1;
                        break;
                    }
                }
            }
        }

        // 输出当前迭代信息
        if (iter % 10 == 0) {
            printf("Iteration %4d: Total screening time = %.2f s (Best: %.2f s)\n",
                   iter, current_params.total_screening_time, best_params.total_screening_time);
        }

        // 检查是否有改进
        if (improved) {
            double improvement = best_params.total_screening_time - current_params.total_screening_time;
            current_params = best_params;
            no_improvement_count = 0;

            // 如果改进很小，考虑减小步长
            if (improvement < TOLERANCE && step_reduction_count < MAX_STEP_REDUCTION) {
                step_v *= REDUCE_FACTOR;
                step_theta *= REDUCE_FACTOR;
                step_tdrop *= REDUCE_FACTOR;
                step_delay *= REDUCE_FACTOR;
                step_reduction_count++;
                printf("Reduced step size. Improvement too small: %.6f\n", improvement);
            }
        } else {
            no_improvement_count++;

            // 如果多次没有改进，减小步长
            if (no_improvement_count >= 5 && step_reduction_count < MAX_STEP_REDUCTION) {
                step_v *= REDUCE_FACTOR;
                step_theta *= REDUCE_FACTOR;
                step_tdrop *= REDUCE_FACTOR;
                step_delay *= REDUCE_FACTOR;
                step_reduction_count++;
                no_improvement_count = 0;
                printf("Reduced step size after %d iterations without improvement\n", 5);
            }

            // 如果步长已经很小且没有改进，认为收敛
            if (step_reduction_count >= MAX_STEP_REDUCTION && no_improvement_count >= 10) {
                printf("Converged after %d iterations\n", iter);
                break;
            }
        }
    }

    // 将最佳参数返回
    *initial_params = best_params;
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
 * 自动化 CSV 加载函数 (无需手动输入)
 * 逻辑：
 * 1. 自动读取 problem5-hill-climbing-strategy.csv 的第一行有效数据。
 * 2. 自动识别该行的无人机 ID (如 "FY2" 则 uav_input_num = 2)。
 * 3. 自动完成角度转换 (Rad->Deg) 和时间累加 (Delta->Absolute)。
 */
void get_user_input(int* uav_input_num, OptimizationParams* initial_params) {
    const std::string filename = "problem5-hill-climbing-strategy.csv";
    
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
        double t1, d1, dt2, d2, dt3, d3;
        
        // 解析格式: 名称, 目标1, 目标2, 目标3, v, rad, t1, d1, dt2, d2, dt3, d3
        // 使用 %[^,] 配合空格跳过逗号前后的空格
        int fields = sscanf(line, "%[^,], %[^,], %[^,], %[^,], %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                            name, m1_str, m2_str, m3_str, 
                            &temp_v, &temp_rad, 
                            &t1, &d1, &dt2, &d2, &dt3, &d3);

        if (fields >= 12) {
            int id = 0;
            if (sscanf(name, "FY%d", &id) == 1) {
                // 存储飞机 ID
                *uav_input_num = id;
                
                // --- A. 解析导弹映射表 ---
                char* m_strs[3] = {m1_str, m2_str, m3_str};
                for (int i = 0; i < 3; i++) {
                    int m_id = 0;
                    if (sscanf(m_strs[i], "M%d", &m_id) == 1) {
                        targets[i] = m_id - 1; // M1 -> 0, M2 -> 1, M3 -> 2
                    } else {
                        printf("警告：解析目标 %s 失败，默认设为 M1\n", m_strs[i]);
                        targets[i] = 0;
                    }
                }

                // --- B. 无人机基本参数 ---
                initial_params->uav_v = temp_v;
                initial_params->uav_theta = temp_rad * (180.0 / M_PI); // 弧度转角度

                // --- C. 烟雾弹时间参数（含链式累加） ---
                // 第一枚弹
                initial_params->t_drop[0] = t1;
                initial_params->delay[0]  = d1;
                // 第二枚弹 (t2 = t1 + dt2)
                initial_params->t_drop[1] = initial_params->t_drop[0] + dt2;
                initial_params->delay[1]  = d2;
                // 第三枚弹 (t3 = t2 + dt3)
                initial_params->t_drop[2] = initial_params->t_drop[1] + dt3;
                initial_params->delay[2]  = d3;

                data_found = true;
                break; 
            }
        }
    }
    fclose(fp);

    if (!data_found) {
        printf("错误：在 %s 中未找到有效的无人机数据或格式不匹配。\n", filename.c_str());
        exit(1);
    }

    // 4. 计算初始总遮蔽时间 (基于解析出的目标映射计算)
    // 注意：需确保 calculate_total_screening_time 内部支持 targets 传入或已全局定义
    initial_params->total_screening_time = calculate_total_screening_time(*uav_input_num, initial_params);

    // 5. 最终确认输出
    printf("\n>>> AUTO-LOADED STRATEGY (HILL CLIMBING) <<<\n");
    printf("UAV Type   : %s (ID: %d)\n", UAV_NAMES[*uav_input_num - 1], *uav_input_num);
    printf("UAV Speed  : %.2f m/s\n", initial_params->uav_v);
    printf("UAV Angle  : %.2f degrees\n", initial_params->uav_theta);
    printf("Target Map : 1->%s, 2->%s, 3->%s\n", 
           MISSILE_NAMES[targets[0]], MISSILE_NAMES[targets[1]], MISSILE_NAMES[targets[2]]);
    
    for (int i = 0; i < GRENADE_COUNT; i++) {
        printf("Grenade %d  : Drop=%.2f s, Delay=%.2f s (Detonate at %.2f s)\n",
               i + 1, initial_params->t_drop[i], initial_params->delay[i], 
               initial_params->t_drop[i] + initial_params->delay[i]);
    }
    printf("Init Score : %.2f s\n", initial_params->total_screening_time);
    printf("--------------------------------------------\n\n");
}

// 打印最终优化结果
void print_final_results(int uav_input_num, const OptimizationParams* params) {
    printf("\n\n=== Final Optimization Results ===\n");
    printf("UAV: %s\n", UAV_NAMES[uav_input_num - 1]);
    printf("Optimized UAV Speed: %.2f m/s\n", params->uav_v);
    printf("Optimized UAV Direction Angle: %.2f degrees\n", params->uav_theta);
    printf("Total Screening Time: %.2f s\n\n", params->total_screening_time);

    for (int i = 0; i < GRENADE_COUNT; i++) {
        double detonate_time = params->t_drop[i] + params->delay[i];
        double intervals[MAX_INTERVALS][2];
        int interval_cnt = 0;

        extract_screening_intervals(uav_input_num - 1, params->uav_v, params->uav_theta,
                                   params->t_drop[i], params->delay[i], targets[i],
                                   detonate_time, intervals, &interval_cnt);

        printf("Grenade %d -> Target %s:\n", i + 1, MISSILE_NAMES[targets[i]]);
        printf("  Drop Time: %.2f s\n", params->t_drop[i]);
        printf("  Detonation Delay: %.2f s\n", params->delay[i]);
        printf("  Detonation Time: %.2f s\n", detonate_time);
        printf("  Individual Screening Time: %.2f s\n",
               calculate_grenade_missile_screening(uav_input_num - 1, params->uav_v,
                                                 params->uav_theta, params->t_drop[i],
                                                 params->delay[i], targets[i]));
        printf("  Screening Intervals: ");
        if (interval_cnt == 0) {
            printf("None\n");
        } else {
            for (int j = 0; j < interval_cnt; j++) {
                printf("[%.2f, %.2f] ", intervals[j][0], intervals[j][1]);
            }
            printf("\n");
        }
        printf("----------------------------------------\n");
    }
}

void solve_problem_5_hill_climbing() {
    srand((unsigned int)time(NULL));
    int uav_input_num;
    OptimizationParams initial_params;

    // 1. 获取用户输入的初始参数
    get_user_input(&uav_input_num, &initial_params);

    // 2. 执行局部爬山法优化
    hill_climbing_optimization(uav_input_num, &initial_params);

    // 3. 打印最终优化结果
    print_final_results(uav_input_num, &initial_params);
}

int main() {
    solve_problem_5_hill_climbing();
    return 0;
}