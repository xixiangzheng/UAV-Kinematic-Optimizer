// src/params.h
#pragma once
#include "config.h"

/**
 * Params 命名空间
 * 作用：作为全局参数控制台。
 * 这里的每一个变量都是 Config 中结构体的实例，
 * 它们在程序运行时是可修改的（非 const），方便调参和 CSV 数据加载。
 */
namespace Params {
    // Problem 2: 随机搜索与蒙特卡洛优化参数
    inline Config::Problem2Params  p2;

    // Problem 3: 差分进化 (DE) 算法参数
    inline Config::Problem3Params  p3;

    // Problem 4: 协同进化 (Co-DE) 算法参数
    inline Config::Problem4Params  p4;

    // Problem 5: 爬山法与网格搜索启发式参数
    inline Config::Problem5Params  p5;
}