// src/config.h
#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Config {
    // ==========================================================
    // 物理引擎与环境常量 
    // ==========================================================
    const double G[3] = {0.0, 0.0, -9.8};
    const double MISSILE_VELOCITY = 300.0;
    const double TARGET_RADIUS = 7.0;

    const double SMOKE_CLOUD_RADIUS = 10.0;
    const double SMOKE_CLOUD_SINK_VELOCITY[3] = {0.0, 0.0, -3.0};
    const double SMOKE_EFFECTIVE_DURATION = 20.0;

    const int GRENADE_COUNT = 3;           // 每架无人机携带烟雾弹数量
    const int MAX_INTERVALS = 50;          // 最大遮蔽区间数量

    // ==========================================================
    // 初始阵位坐标 (三维矢量) [x, y, z]
    // ==========================================================
    const double FAKE_TARGET_POS[3] = {0.0, 0.0, 0.0};
    const double TRUE_TARGET_POS_BOTTOM_CENTER[3] = {0.0, 200.0, 0.0};
    const double TRUE_TARGET_POS_TOP_CENTER[3]    = {0.0, 200.0, 10.0};

    const double MISSILE_INITIAL_POS[3][3] = {{20000.0, 0.0, 2000.0}, {19000.0, 600.0, 2100.0}, {18000.0, -600.0, 1900.0}};
    const double *MISSILE_M1_INITIAL_POS = MISSILE_INITIAL_POS[0];

    const double UAV_INITIAL_POS[5][3] = {{17800.0,0.0,1800.0}, {12000.0,1400.0,1400.0}, {6000.0,-3000.0,700.0}, {11000.0,2000.0,1800.0}, {13000.0,-2000.0,1300.0}};
    const double* UAV_FY1_INITIAL_POS = UAV_INITIAL_POS[0];

    const double DEFAULT_UAV_SPEED = 120.0;
    const double DEFAULT_T_DROP = 1.5;
    const double DEFAULT_DETONATION_DELAY = 3.6;

    const double SIM_TIME_STEP = 0.0001; 
    const double SIM_PRINT_INTERVAL = 0.01;

    // ==========================================================
    // 3. 算法参数结构体定义 (用于 Params.h 实例化)
    // ==========================================================

    // Problem 2 特有的随机搜索与优化参数
    struct Problem2Params {
        // 1. 蒙特卡洛搜索边界
        double OPT_TIME_STEP = 0.001; 
        double OPT_SPEED_MIN = 70.0;
        double OPT_SPEED_MAX = 140.0;
        double OPT_T_DROP_MIN = 0.0;
        double OPT_T_DROP_MAX = 30.0;
        double OPT_DELAY_MIN  = 0.0;
        double OPT_DELAY_MAX  = 10.0;

        // 2. 迭代次数控制
        int GLOBAL_SEARCH_ITERATIONS = 10000; 
        int LOCAL_SEARCH_ROUNDS = 4;           
        int ITERATIONS_PER_ROUND = 2500;       

        // 3. 局部搜索启发式参数
        double LOCAL_SEARCH_INITIAL_RANGE = 0.2; 
        double LOCAL_SEARCH_DECAY_RATE = 0.6;    
    };

    // Problem 3 特有的差分进化 (DE) 算法参数
    struct Problem3Params {
        int DE_POPULATION_SIZE = 200;   
        int DE_MAX_GENERATIONS = 700;   
        double DE_F = 0.8;              
        double DE_CR = 0.9;   
        double DE_TIME_STEP = 0.01;          

        // 搜索边界 (mins 和 maxs)
        double DE_MINS[8] = {70.0, 0.0, 0.1, 0.1, 1.0, 0.1, 1.0, 0.1};
        double DE_MAXS[8] = {140.0, 2.0 * M_PI, 20.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    };

    // Problem 4 特有的协同进化 (Co-DE) 算法参数
    struct Problem4Params {
        int CO_POPULATION_SIZE = 100;    
        int CO_MAX_GENERATIONS = 500;    
        double CO_F = 0.8;               
        double CO_CR = 0.9;              
        double PARTICIPATION_BONUS = 2.0; 
        double CO_TIME_STEP = 0.01;

        // 协同进化的搜索边界 [速度, 角度, 投放时间, 延迟]
        double CO_MINS[4] = {70.0, 0.0, 0.1, 0.1};
        double CO_MAXS[4] = {140.0, 2.0 * M_PI, 30.0, 10.0};
    };

    // Problem 5 特有的局部搜索与网格启发式参数
    struct Problem5Params {
        // 1. 离散搜索步数 (Grid Search)
        int DROP_STEPS = 80;              
        int DELAY_STEPS = 50;             

        // 2. 爬山法控制参数 (Hill Climbing)
        int MAX_ITERATIONS = 1000;        
        double TOLERANCE = 0.001;         
        
        // 3. 搜索步长定义 (Step Sizes)
        double STEP_SIZE_V = 1.0;         
        double STEP_SIZE_THETA = 1.0;     
        double STEP_SIZE_TDROP = 0.5;     
        double STEP_SIZE_DELAY = 0.2;     
        
        // 4. 自适应步长缩减
        double REDUCE_FACTOR = 0.5;       
        int MAX_STEP_REDUCTION = 5;       

        // 5. 搜索边界 (Search Boundaries)
        double MIN_DROP = 0.0;
        double MAX_DROP = 50.0;
        double MIN_DELAY = 0.1;
        double MAX_DELAY = 20.0;
        double STEP_TIME_STEP = 0.001;
    };
}