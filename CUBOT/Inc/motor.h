#include "fdcan.h"
#include "pid.h"

#ifndef MOTOR_H
#define MOTOR_H

/**
  * @brief CAN发送函数
  */
uint8_t CAN_Send(FDCAN_HandleTypeDef* can,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4 );

uint8_t CAN_Send_GM6020_volt(FDCAN_HandleTypeDef* can,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4 );

uint8_t CAN_Open(FDCAN_HandleTypeDef* can);

/**
 * @brief 描述电机状态
 * 
 * @param ecd 电机编码器值,范围0-8191,对应0-360度,表示转子机械角度
 * @param speed_rpm 电机速度,单位rpm
 * @param given_current 电机实际转矩电流
 * @param temperate 电机温度
 * @param last_ecd 上次电机编码器值
 * @param angle_armature 转子角度值,范围-180°-180°
 * @param last_angle_armature 上次转子角度值
 * @param turn 转子转过圈数,范围-19到19
 * @param angle 转轴角度值,范围-180°-180°
 * @param last_angle 上次转轴角度值
 */
typedef struct 
{ 
  uint16_t ecd; 
  int16_t speed_rpm; 
  int16_t given_current; 
  uint8_t temperate; 
  int16_t last_ecd;
  float angle_armature;
  int8_t turn;
  float angle;
  float last_angle_armature;
  float last_angle;
} motor_measure_t; 

typedef struct 
{ 
  uint16_t ecd; 
  int16_t speed_rpm; 
  int16_t given_current; 
  uint8_t temperate; 
  int16_t last_ecd;
  float angle;
  float last_angle;
} motor_measure_gm6020_t;

/*CAN接收的ID*/
typedef enum
{
	//add by langgo
	CAN_3508Moto1_ID = 0x201,
	CAN_3508Moto2_ID = 0x202,
	CAN_3508Moto3_ID = 0x203,
	CAN_3508Moto4_ID = 0x204,
}CAN_Message_ID;

typedef enum
{
  //add by langgo
  CAN_GM6020Moto1_ID = 0x205,
  CAN_GM6020Moto2_ID = 0x206,
  CAN_GM6020Moto3_ID = 0x207,
  CAN_GM6020Moto4_ID = 0x208,
}CAN_GM6020_Message_ID;

void GM6020_init(void);
void GM6020_speed_control(FDCAN_HandleTypeDef* can, int16_t speed1,int16_t speed2,int16_t speed3,int16_t speed4);
void GM6020_ecd_control(FDCAN_HandleTypeDef* can, uint16_t ecd1,uint16_t ecd2,uint16_t ecd3,uint16_t ecd4);

#define DEBUG_PID_OUTPUT 1
#if DEBUG_PID_OUTPUT 

int error_print(void);

int measure_print(void);

int target_d1(void);

#endif

#endif /* MOTOR_H */
