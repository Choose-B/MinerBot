# pragma once
#include "main.h"
#include <stdarg.h>

#define HIGHEST_DERIVATIVE 4

/**
 * @brief PID控制器结构体
 * 
 * @param frequency 采样频率
 * @param target 目标值
 * @param measure 测量值
 * @param last_measure 上次测量值
 * @param error 误差值
 * @param p 比例项
 * @param k_p 比例系数
 * @param i 积分项
 * @param k_i 积分系数
 * @param i_limit 积分项限幅值
 * @param episilon 积分分离阈值
 * @param beta 积分分离开关
 * @param d 微分项
 * @param k_d 微分系数
 * @param max_output 最大输出
 * @param feedforward 前馈项总和
 */
typedef struct {
    int frequency;//采样频率

    int target[HIGHEST_DERIVATIVE];//目标值,target[0]为当前目标值，下标越大目标值越旧

    float measure;//测量值
    float last_measure;//上次测量值

    float error;//误差值
    
    float p;//比例项
    float k_p;//比例系数

    float i;//积分项
    float k_i;//积分系数
    float i_limit;//积分项限幅值
    int episilon;//积分分离阈值
    uint8_t beta;//积分分离开关

    float d;//微分项
    float k_d;//微分系数

    float feedforward;//前馈项总和

    float max_output;//最大输出
}PID;

/**
 * @brief   初始化PID结构体
 * 
 * @param pid 指向需要初始化的pid对象   
 * @param k_p 比例系数
 * @param k_i 积分系数
 * @param k_d 微分系数
 * @param target 目标值
 * @param frequency 采样频率
 * @param max_output 最大输出
 * @param i_limit 积分项最大值
 * @param episilon 积分分离阈值
 * @return 是否初始化成功 1-成功 0-失败
 */
uint8_t PID_Init( PID* pid,
                float k_p,
                float k_i,
                float k_d,
                float target,
                int frequency,
                float max_output,
                float i_limit,
                float episilon
            );

/**
 * @brief 计算控制量
 * 
 * @param pid 指向对某一被控量进行控制的pid对象
 * @param measure 反馈值
 * @param n 传入的前馈函数数量，如果没有就填0
 * @param func_list 类型要求为 float(*)(int) 
 * @return 控制量 
 */
float PID_Calc( PID* pid, float measure, int n, ... );

/**
 * @brief 传入当前目标值
 * 
 * @param pid 指向对某一被控量进行控制的pid对象
 * @param target_0 目标值
 */
void PID_target(PID* pid, int target_0);

/**
 * @brief 复制pid对象，便于批量初始化
 * 
 * @param dest 被复制对象
 * @param src 源对象
 */
void PID_Copy(PID* dest, PID* src);
