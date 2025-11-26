#include "dipan.h"

const uint16_t vxy_MAX = 1000;
const uint16_t w_MAX = 500;

PID motor_speed_pid_1;
PID motor_speed_pid_2;
PID motor_speed_pid_3;
PID motor_speed_pid_4;

void Dipan_Init(void){
    PID_Init(&motor_speed_pid_1,0.6f,0.4f,0.0f, 0,1000,16384,500.0f,600);
    PID_Init(&motor_speed_pid_2,0.6f,0.4f,0.0f, 0,1000,16384,500.0f,600);
    PID_Init(&motor_speed_pid_3,0.6f,0.4f,0.0f, 0,1000,16384,500.0f,600);
    PID_Init(&motor_speed_pid_4,0.6f,0.4f,0.0f, 0,1000,16384,500.0f,600);
}

/**
 * @brief 麦克拉姆论轮底盘运动学计算
 * 
 * @attention 支持麻神www.bilibili.com/video/BV1toH6ekEfJ
 * 
 * @param vx 速度x分量
 * @param vy 速度y分量
 * @param w 角速度
 * @return 发送是否成功
 */
uint8_t Dipan_Mecanum(float vx, float vy, float w){
    vx *= vxy_MAX;
    vy *= vxy_MAX;
    w  *= w_MAX;

    float v1 =  vx + vy + w ;
    float v2 = -vx + vy + w ;  
    float v3 = -vx - vy + w ;  
    float v4 =  vx - vy + w ;

    PID_target(&motor_speed_pid_1, v1*(3591.0f/187.0f));
    PID_target(&motor_speed_pid_2, v2*(3591.0f/187.0f));
    PID_target(&motor_speed_pid_3, v3*(3591.0f/187.0f));
    PID_target(&motor_speed_pid_4, v4*(3591.0f/187.0f));

    float m1 = PID_Calc(&motor_speed_pid_1,v1,0);
    float m2 = PID_Calc(&motor_speed_pid_2,v2,0);
    float m3 = PID_Calc(&motor_speed_pid_3,v3,0);
    float m4 = PID_Calc(&motor_speed_pid_4,v4,0);

    return 0;
}
