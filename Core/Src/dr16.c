/**@file  dr16.c
* @brief    设备层
* @details  主要包括构建串口管理器，提供串口初始化和用户回调重定义
* @author      RyanJiao  any question please send mail to 1095981200@qq.com
* @date        2021-10-9
* @version     V1.0
* @copyright    Copyright (c) 2021-2121  中国矿业大学CUBOT战队
**********************************************************************************
* @attention
* 硬件平台: STM32H750VBT \n
* SDK版本：-++++
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>创建初始版本
* </table>
*
**********************************************************************************
==============================================================================
												How to use this driver  
==============================================================================


	********************************************************************************
	* @attention
	* 硬件平台: STM32H750VBT \n
	* SDK版本：-++++
	* if you had modified this file, please make sure your code does not have many
	* bugs, update the version NO., write dowm your name and the date, the most
	* important is make sure the users will have clear and definite understanding 
	* through your new brief.
	********************************************************************************
*/

#include "dr16.h"
#include <stdlib.h>

RC_Ctrl rc_Ctrl={
	.isUnpackaging = 0,
	.isOnline = 0
	
};

/**
	* @brief  创建dr16的接收机缓存数组
	*/
uint8_t DR16_recData[DR16_rxBufferLengh];

int Check_receiver=31;


//将下面的代码加入到定时器中断中，一旦isOnline为0，就把电机输出置零（必须加，不然可能会疯车）
/**	if(Check_receiver > 100)
	Check_receiver = 100;
if(Check_receiver>30)
	rc_Ctrl.isOnline = 0;
else
	rc_Ctrl.isOnline = 1;
*/

/**
  * @brief  初始化接收机数据类型的数据，将杆量和按键信息归零
  */
void DR16Init(RC_Ctrl* RC_Ctl)
{
	RC_Ctl->rc.ch0=1024;
	RC_Ctl->rc.ch1=1024;
	RC_Ctl->rc.ch2=1024;
	RC_Ctl->rc.ch3=1024;
	RC_Ctl->rc.s1=3;
	RC_Ctl->rc.s2=3;
	RC_Ctl->rc.sw=1024;
	RC_Ctl->mouse.x=0;
	RC_Ctl->mouse.y=0;
	RC_Ctl->mouse.z=0;		
	RC_Ctl->key_Q_flag=0;
	RC_Ctl->key_E_flag=0;    //< 上电关弹舱
	RC_Ctl->key_R_flag=0;
	RC_Ctl->key_F_flag=0;
	RC_Ctl->key_G_flag=0;
	RC_Ctl->key_Z_flag=0;
	RC_Ctl->key_X_flag=0;
	RC_Ctl->key_C_flag=0;
	RC_Ctl->key_V_flag=0;
	RC_Ctl->key_B_flag=0;
	RC_Ctl->key_ctrl_flag=0;
	RC_Ctl->Chassis_Y_Integ=0;//斜坡积分变量
	RC_Ctl->Chassis_X_Integ=0;
	RC_Ctl->ShootNumber=1;
	RC_Ctl->Cruise_Mode = 0;
}


uint8_t correct_num=0;
/**
  * @brief  创建dr16的接收机缓存数组, 并对全局变量rc_Ctrl赋值，以供其他函数调用
  */
void DR16_DataUnpack(RC_Ctrl* rc_ctrl, uint8_t * recBuffer)
{ 
//	tim14_FPS.Receiver_cnt++;
	rc_ctrl->isUnpackaging = 1;					//< 解算期间不允许读取数据
		
	correct_num=0;
	if(((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff)<=1684 && ((recBuffer[0] | (recBuffer[1] << 8)) & 0x07ff)>=364)
		correct_num++;
	if((((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff)<=1684 && (((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff)>=364)
		correct_num++;
	if((((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff)<=1684 && (((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff)>=364)
		correct_num++;
	if((((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff)<=1684 && (((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff)>=364)
		correct_num++;
	if((((recBuffer[5] >> 4)& 0x000C) >> 2)==1 || (((recBuffer[5] >> 4)& 0x000C) >> 2)==2 || (((recBuffer[5] >> 4)& 0x000C) >> 2)==3)
		correct_num++;
	if(((recBuffer[5] >> 4)& 0x0003)==1 || ((recBuffer[5] >> 4)& 0x0003)==2 || ((recBuffer[5] >> 4)& 0x0003)==3)
		correct_num++;
	if(correct_num==6)																																												//< 数据校验通过
	{
		rc_ctrl->rc.ch0 = (recBuffer[0]| (recBuffer[1] << 8)) & 0x07ff; 																					//< Channel 0   高8位与低3位
		rc_ctrl->rc.ch1 = ((recBuffer[1] >> 3) | (recBuffer[2] << 5)) & 0x07ff; 																	//< Channel 1   高5位与低6位
		rc_ctrl->rc.ch2 = ((recBuffer[2] >> 6) | (recBuffer[3] << 2) |(recBuffer[4] << 10)) & 0x07ff; 						//< Channel 2
		rc_ctrl->rc.ch3 = ((recBuffer[4] >> 1) | (recBuffer[5] << 7)) & 0x07ff; 																	//< Channel 3
		rc_ctrl->rc.s1 = ((recBuffer[5] >> 4)& 0x000C) >> 2; 																											//!< Switch left
		rc_ctrl->rc.s2 = ((recBuffer[5] >> 4)& 0x0003); 				//提取数组5的3，4位																									//!< Switch right
		rc_ctrl->rc.sw=(uint16_t)(recBuffer[16]|(recBuffer[17]<<8))&0x7ff;

		if((rc_ctrl->rc.ch0>1020)&&(rc_ctrl->rc.ch0<1028))          //遥控器零飘
			rc_ctrl->rc.ch0=1024;
		if((rc_ctrl->rc.ch1>1020)&&(rc_ctrl->rc.ch1<1028))
			rc_ctrl->rc.ch1=1024;
		if((rc_ctrl->rc.ch2>1020)&&(rc_ctrl->rc.ch2<1028))
			rc_ctrl->rc.ch2=1024;
		if((rc_ctrl->rc.ch3>1020)&&(rc_ctrl->rc.ch3<1028))
			rc_ctrl->rc.ch3=1024;

			/***********鼠标信息*************/
		rc_ctrl->mouse.x = recBuffer[6]  | (recBuffer[7] << 8);                       //< Mouse X axis
		rc_ctrl->mouse.y = recBuffer[8]  | (recBuffer[9] << 8);                       //< Mouse Y axis
		rc_ctrl->mouse.z = recBuffer[10] | (recBuffer[11] << 8);                      //< Mouse Z axis
		rc_ctrl->mouse.press_l = recBuffer[12];                                       //< Mouse Left Is Press ?
		rc_ctrl->mouse.press_r = recBuffer[13];                                       //< Mouse Right Is Press ?

		if(rc_ctrl->mouse.x>25000)   rc_ctrl->mouse.x=25000;     																												//< 限幅
		if(rc_ctrl->mouse.x<-25000)  rc_ctrl->mouse.x=-25000;
		if(rc_ctrl->mouse.y>25000)   rc_ctrl->mouse.y=25000;
		if(rc_ctrl->mouse.y<-25000)  rc_ctrl->mouse.y=-25000;

		rc_ctrl->keyboard.v = recBuffer[14]| (recBuffer[15] << 8);  									//< 共16个按键值

		rc_ctrl->key_W=recBuffer[14]&0x01;	
		rc_ctrl->key_S=(recBuffer[14]>>1)&0x01;					
		rc_ctrl->key_A=(recBuffer[14]>>2)&0x01;
		rc_ctrl->key_D=(recBuffer[14]>>3)&0x01;					
		rc_ctrl->key_B=(recBuffer[15]>>7)&0x01;
		rc_ctrl->key_V=(recBuffer[15]>>6)&0x01;				
		rc_ctrl->key_C=(recBuffer[15]>>5)&0x01;
		rc_ctrl->key_X=(recBuffer[15]>>4)&0x01;					
		rc_ctrl->key_Z=(recBuffer[15]>>3)&0x01;					
		rc_ctrl->key_G=(recBuffer[15]>>2)&0x01;			
		rc_ctrl->key_F=(recBuffer[15]>>1)&0x01;
		rc_ctrl->key_R=(recBuffer[15])&0x01;													
		rc_ctrl->key_E=(recBuffer[14]>>7)&0x01;
		rc_ctrl->key_Q=(recBuffer[14]>>6)&0x01;
		rc_ctrl->key_ctrl=(recBuffer[14]>>5)&0x01;
		rc_ctrl->key_shift=(recBuffer[14]>>4)&0x01;


	Check_receiver = 0;

	}
	else{}
	rc_ctrl->isUnpackaging = 0;					//< 解算完成标志位，允许读取
}



