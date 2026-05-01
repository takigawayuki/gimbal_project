#ifndef __ZhangDaTou_H
#define __ZhangDaTou_H
#include "main.h"
#include "usart.h"

typedef struct
{
	UART_HandleTypeDef *huart;
	uint8_t id;
	uint8_t mod;
	float setSpeed;			//度每秒
	float setPosition;		//转每秒
	float Speed;
	float Position;
	uint8_t ready;
	uint32_t microStep;
	uint32_t reduction_ratio;//减速比
}ZDTMotor_Typedef;


extern ZDTMotor_Typedef pitchmotor;
extern ZDTMotor_Typedef yawmotor;


//电机任务函数
void ZhangDaTou_Task(void);
//电机串口数据解析
void ZhangDaTou_DataParm(uint8_t* Data,ZDTMotor_Typedef* object);
//速度模式控制
void ZhangDaTou_Speedctr(ZDTMotor_Typedef* object,float SpeedVal);
//位置速度控制
void ZhangDaTou_PositionSpeedctr(ZDTMotor_Typedef* object,float SpeedVal,float PositionVal);
//获取速度
float ZhangDaTou_getSpeedDate(ZDTMotor_Typedef* object);
//获取位置
float ZhangDaTou_getPositionDate(ZDTMotor_Typedef* object);
//设置变量所代表的电机id,
void ZhangDaTou_init(ZDTMotor_Typedef* object,uint8_t id);
void ZhangDaTou_SetUart(ZDTMotor_Typedef* object, UART_HandleTypeDef *huart);

#endif
