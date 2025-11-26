# include "motor.h"

void get_motor_measure(motor_measure_t* ptr, uint8_t* data)
{
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    ptr->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);
    ptr->given_current = (int16_t)((data)[4] << 8 | (data)[5]);
    ptr->temperate = (data)[6];

    ptr->last_angle_armature = ptr->angle_armature;
    ptr->angle_armature = ptr->ecd / 8191.0f * 360.0f - 180.0f;

    // 计算转过的圈数
    // 转轴的空载转速为482rpm,转子的空载转速为482rpm/187*3591=9255.94,发送信号的频率为1kHz,每1ms转子最多转过0.154圈,对应编码器值为1263.59
    // 因此每次读取时编码器值不可能跳变超过1263.59,转过一圈则会出现前后两次差超过6500的情况
    // 这里设置阈值为6500来判断是否转过一圈
    if (fabs(ptr->ecd - ptr->last_ecd) > 6500) {
      if (ptr->ecd > ptr->last_ecd) {
          ptr->turn--;
      }
      else {
          ptr->turn++;
      }
    }
    if (fabs(ptr->turn)>19){
        ptr->turn=0;
    }

    ptr->last_angle = ptr->angle;
    ptr->angle = ptr->ecd / 8191.0f * 360.0f / (3591.0f/187.0f) + ptr->turn * 360.0f / (3591.0f/187.0f);//减速比3591:187
    ptr->angle -= 180.0f;
    ptr->angle = ptr->angle < -180.0f ? ptr->angle + 360.0f : ptr->angle;
    ptr->angle = ptr->angle > 180.0f ? ptr->angle - 360.0f : ptr->angle;
}

void get_motor_gm6020_measure(motor_measure_gm6020_t* ptr, uint8_t* data)
{
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    ptr->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);
    ptr->given_current = (int16_t)((data)[4] << 8 | (data)[5]);
    ptr->temperate = (data)[6];

    ptr->last_angle = ptr->angle;
    ptr->angle = ptr->ecd / 8191.0f * 360.0f - 180.0f;

}


uint8_t chassis_can_send_data[8];
/**
  * @brief CAN发送函数，电流值-16384 ~ 16384对应电流值-20 ~ 20A
  * @param can: CAN句柄
  * @param motor1: 电机1电流
  * @param motor2: 电机2电流
  * @param motor3: 电机3电流
  * @param motor4: 电机4电流
  * @retval 发送状态，1成功，0失败
  */
uint8_t CAN_Send(FDCAN_HandleTypeDef* can,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4 )
{
	FDCAN_TxHeaderTypeDef txHeader;
	txHeader.Identifier = (uint32_t)0x200;//0x200是电机1-4,0x1FF是电机5-8
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0x00;
	
  chassis_can_send_data[0] = motor1 >> 8; 
  chassis_can_send_data[1] = motor1; 
  chassis_can_send_data[2] = motor2 >> 8; 
  chassis_can_send_data[3] = motor2; 
  chassis_can_send_data[4] = motor3 >> 8; 
  chassis_can_send_data[5] = motor3; 
  chassis_can_send_data[6] = motor4 >> 8; 
  chassis_can_send_data[7] = motor4; 

    if(HAL_FDCAN_AddMessageToTxFifoQ(can, &txHeader,chassis_can_send_data) != HAL_OK)
    {
        return 0;
    }
    else
    {
        return 1;
    }		
}

uint8_t CAN_Send_GM6020_volt(FDCAN_HandleTypeDef* can,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4 )
{
	FDCAN_TxHeaderTypeDef txHeader;
	txHeader.Identifier = (uint32_t)0x1FF;//0x1FE是电机1-4,0x2FE是电机5-8
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0x00;
	
  chassis_can_send_data[0] = motor1 >> 8; 
  chassis_can_send_data[1] = motor1; 
  chassis_can_send_data[2] = motor2 >> 8; 
  chassis_can_send_data[3] = motor2; 
  chassis_can_send_data[4] = motor3 >> 8; 
  chassis_can_send_data[5] = motor3; 
  chassis_can_send_data[6] = motor4 >> 8; 
  chassis_can_send_data[7] = motor4; 

    if(HAL_FDCAN_AddMessageToTxFifoQ(can, &txHeader,chassis_can_send_data) != HAL_OK)
    {
        return 0;
    }
    else
    {
        return 1;
    }		
}


/**
 * @brief 初始化滤波器
 * 
 * @param can CAN句柄
 */
uint8_t CAN_Open(FDCAN_HandleTypeDef* can) 
{ 
    FDCAN_FilterTypeDef filter;                   	//< 声明局部变量 can过滤器结构体
	filter.IdType       = FDCAN_STANDARD_ID;       	//< id设置为标准id
	filter.FilterIndex  = 0;                      	//< 设值筛选器的编号，标准id选择0-127
	filter.FilterType   = FDCAN_FILTER_MASK;       	//< 设置工作模式为掩码模式
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 	//< 将经过过滤的数据存储到 fifo0
	filter.FilterID1    = 0x000;                   	//< 筛选器的id
	filter.FilterID2    = 0x000;
	
	HAL_FDCAN_ConfigFilter(can, &filter);   //< 配置过滤器	
  HAL_StatusTypeDef check_start_can;
  check_start_can = HAL_FDCAN_Start(can);                   //< 使能can
    //该check来测试can控制器是否使能，可以把该赋值去掉	
	HAL_FDCAN_ActivateNotification(can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使能fifo0接收到新信息中断
  return check_start_can == HAL_OK?1:0;
}

motor_measure_t motor_chassis[4] = {0};//4 chassis motor
motor_measure_gm6020_t motor_gm6020[4] = {0};//4 gm6020 motor

/**
 * @brief CAN中断回调函数
 * 
 * @param hcan CAN句柄
 * @param RxFifo0ITs 
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan,uint32_t RxFifo0ITs) 
{ 
	if(hcan==&hfdcan2){
    FDCAN_RxHeaderTypeDef rx_header; 
    uint8_t rx_data[8]; 

    HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rx_header, rx_data); 

    switch (rx_header.Identifier) { 
      case CAN_3508Moto1_ID: 
      case CAN_3508Moto2_ID: 
      case CAN_3508Moto3_ID: 
      case CAN_3508Moto4_ID: { 
        static uint8_t i = 0; 
        //get motor id 
        i = rx_header.Identifier - CAN_3508Moto1_ID; 
        get_motor_measure(&motor_chassis[i], rx_data); 
          
        break; 
      } 
      default: { 
        break; 
      } 
    } 
	}
  if(hcan==&hfdcan1){
    FDCAN_RxHeaderTypeDef rx_header; 
    uint8_t rx_data[8]; 

    HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rx_header, rx_data); 

    switch (rx_header.Identifier) { 
      case CAN_GM6020Moto1_ID: 
      case CAN_GM6020Moto2_ID: 
      case CAN_GM6020Moto3_ID: 
      case CAN_GM6020Moto4_ID: { 
        static uint8_t i = 0; 
        //get motor id 
        i = rx_header.Identifier - CAN_GM6020Moto1_ID; 
        get_motor_gm6020_measure(&motor_gm6020[i], rx_data); 
          
        break; 
      } 
      default: { 
        break; 
      } 
    }
  }
} 

PID gm6020_ecd_pid_1,gm6020_ecd_pid_2,gm6020_ecd_pid_3,gm6020_ecd_pid_4;
PID gm6020_speed_pid_1,gm6020_speed_pid_2,gm6020_speed_pid_3,gm6020_speed_pid_4;

void GM6020_init(void){
    PID ecd_pid_tmp;
    PID_Init(&ecd_pid_tmp, 0.6f, 0.25f, 0.0f, 0.0f, 1000, 320.0f, 300.0f, 30.0f);
    PID_Copy(&gm6020_ecd_pid_1, &ecd_pid_tmp);
    PID_Copy(&gm6020_ecd_pid_2, &ecd_pid_tmp);
    PID_Copy(&gm6020_ecd_pid_3, &ecd_pid_tmp);
    PID_Copy(&gm6020_ecd_pid_4, &ecd_pid_tmp);

    PID speed_pid_tmp;
    PID_Init(&speed_pid_tmp, 50.0f, 50.0f, 0.0f, 0.0f, 1000, 25000.0f, 2000.0f, 45.0f);
    PID_Copy(&gm6020_speed_pid_1, &speed_pid_tmp);
    PID_Copy(&gm6020_speed_pid_2, &speed_pid_tmp);
    PID_Copy(&gm6020_speed_pid_3, &speed_pid_tmp);
    PID_Copy(&gm6020_speed_pid_4, &speed_pid_tmp);
}

/**
 * @brief GM6020摩擦力前馈。使用线性拟合，R^2超过0.9999
 * @param pid 
 * @return 应有电压值
 */
float GM6020_speed_feedforward_friction(PID* pid){
  float r = 0;
  if (pid->target[0] > 0.5){
    r = pid->target[0]*69.392 + 467.9097;
  }
  if (pid->target[0] < -0.5){
    r = pid->target[0]*69.392 - 467.9097;
  }
  if ( r >= -468 && r <= 468){
    r = 0;
  }
  return r;
}

void GM6020_speed_control(FDCAN_HandleTypeDef* can, int16_t speed1,int16_t speed2,int16_t speed3,int16_t speed4)
{
    PID_target(&gm6020_speed_pid_1, speed1);
    PID_target(&gm6020_speed_pid_2, speed2);
    PID_target(&gm6020_speed_pid_3, speed3);
    PID_target(&gm6020_speed_pid_4, speed4);

    CAN_Send_GM6020_volt(can,
                            (int16_t)PID_Calc(&gm6020_speed_pid_1,motor_gm6020[0].speed_rpm,1,GM6020_speed_feedforward_friction),
                            (int16_t)PID_Calc(&gm6020_speed_pid_2,motor_gm6020[1].speed_rpm,1,GM6020_speed_feedforward_friction),
                            (int16_t)PID_Calc(&gm6020_speed_pid_3,motor_gm6020[2].speed_rpm,1,GM6020_speed_feedforward_friction),
                            (int16_t)PID_Calc(&gm6020_speed_pid_4,motor_gm6020[3].speed_rpm,1,GM6020_speed_feedforward_friction));    
}

float Zero_Crossing(int ecd, int32_t target){
  if ( fabs(ecd - target) > 4095 ){
    if ( ecd > target ){
      return ecd - 8191;
    }
    else {
      return ecd + 8191;
    }
  }
  return ecd;
}

float GM6020_ecd_feedforward_d1(PID* pid){
//  int total_weight = 0;
  int d = pid->target[0] - pid->target[1];
//  int d2 = pid->target[1] - pid->target[2];
//  int d3 = pid->target[2] - pid->target[3];
//  float _d = d1 * 0.5 + d2 * 0.3 + d3 * 0.2; // 滤波
//  float d = 0;
//  if ( fabs(d-d1) < 200 ){
//    d += d1 * 5;
//    total_weight += 5;
//  }
//  if ( fabs(d-d2) < 200){
//    d += d2 * 3;
//    total_weight += 3;
//  }
//  if ( fabs(d-d3) < 200 ){
//    d += d3 * 2;
//    total_weight += 2;
//  }
//  d = d/total_weight;

  float r = 0;
  if (d > 1){
    r = d * 7.317f - 0.429f;
  }
  if (d < -1){
    r = d * 7.317f + 0.429f;
  }
  if ( fabs(r) >= 1000 ){
    if (r > 0){
      r = 1000;
    }
    else {
      r = -1000;
    }
  }
  return r;
}

void GM6020_ecd_control(FDCAN_HandleTypeDef* can, uint16_t ecd1,uint16_t ecd2,uint16_t ecd3,uint16_t ecd4)
{
    PID_target(&gm6020_ecd_pid_1, ecd1);
    PID_target(&gm6020_ecd_pid_2, ecd2);
    PID_target(&gm6020_ecd_pid_3, ecd3);
    PID_target(&gm6020_ecd_pid_4, ecd4);

    GM6020_speed_control(can,
                            (int16_t)PID_Calc(&gm6020_ecd_pid_1,Zero_Crossing(motor_gm6020[0].ecd, ecd1),1,GM6020_ecd_feedforward_d1),
                            (int16_t)PID_Calc(&gm6020_ecd_pid_2,Zero_Crossing(motor_gm6020[1].ecd, ecd1),1,GM6020_ecd_feedforward_d1),
                            (int16_t)PID_Calc(&gm6020_ecd_pid_3,Zero_Crossing(motor_gm6020[2].ecd, ecd1),1,GM6020_ecd_feedforward_d1),
                            (int16_t)PID_Calc(&gm6020_ecd_pid_4,Zero_Crossing(motor_gm6020[3].ecd, ecd1),1,GM6020_ecd_feedforward_d1));
}

#if DEBUG_PID_OUTPUT 

int error_print(void){
  return gm6020_ecd_pid_1.error;
}

int measure_print(void){
  return gm6020_ecd_pid_1.measure;
}

int target_d1(void){
  return (gm6020_ecd_pid_1.target[0]-gm6020_ecd_pid_1.target[1]);
}

#endif
