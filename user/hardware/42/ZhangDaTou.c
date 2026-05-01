#include "ZhangDaTou.h"
#include "Emm_V5.h"
#include "usart.h"
#include "string.h"
ZDTMotor_Typedef pitchmotor={
.huart = &huart1,
.id =1,
.mod=1,
.setSpeed =100,
.reduction_ratio = 1,
.microStep =256,
};
ZDTMotor_Typedef yawmotor={
.huart = &huart3,
.id =2,
.mod=1,
.reduction_ratio =1,
.setSpeed =100,
.microStep =256,
};

uint32_t swap_endian_32(uint32_t val) {
    return ((val >> 24) & 0x000000FF) |  // 移动最高字节到最低位
           ((val >> 8)  & 0x0000FF00) |  // 移动次高字节到次低位
           ((val << 8)  & 0x00FF0000) |  // 移动次低字节到次高位
           ((val << 24) & 0xFF000000);    // 移动最低字节到最高位
}
uint16_t swap_endian_16(uint16_t val) {
    return ((val >> 8) & 0x00FF) |  // 移动高字节到低位
           ((val << 8) & 0xFF00);    // 移动低字节到高位
}

void ZhangDaTou_DataParm(uint8_t* Data,ZDTMotor_Typedef* object)
{
	if(Data[0]==object->id)
	{
		switch(Data[1])
		{
			case 0x35://速度
				if(Data[6-1]==0x6B)
				{
					static signed char sign=1;
					static uint16_t temp;
					if(Data[3-1]==1)
					{
						sign =-1;
					}
					else
					{
						sign =1;
					}
					memcpy(&temp, &Data[3], sizeof(uint16_t));
					object->Speed = swap_endian_16(temp)*sign*6;
				}
				break;
			case 0x36://位置
				if(Data[7]==0x6B)
				{
					static signed char sign=1;
					static uint32_t temp;
					if(Data[3-1]==1)
					{
						sign =-1;
					}
					else
					{
						sign =1;
					}
					memcpy(&temp, &Data[3], sizeof(uint32_t));
					object->Position=swap_endian_32(temp)*360.0f/65536.0f/object->reduction_ratio *sign;
				}
				break;
			default:
				break;
		}
	}
}

void ZhangDaTou_Control(ZDTMotor_Typedef* object)
{
	if(object->mod==0)//速度模式
	{
		if(object->setSpeed>0)
		{
			Emm_V5_Vel_Control(object->huart, object->id,0,(uint16_t)(object->setSpeed/6.0f),0,0);
		}
		else
		{
			Emm_V5_Vel_Control(object->huart, object->id,1,(uint16_t)(-object->setSpeed/6.0f),0,0);
		}
	}	
	else//速度位置模式
	{
		if(object->setSpeed<0)
		{
			object->setSpeed = -object->setSpeed;
		}
		if(object->setPosition>=0)
		{
			Emm_V5_Pos_Control(object->huart, object->id,0,(uint16_t)(object->setSpeed/6.0f),0,(uint32_t)(object->setPosition*(3200.0f*object->microStep/16*object->reduction_ratio)/360.0f),true,0);
		}
		else
		{
			Emm_V5_Pos_Control(object->huart, object->id,1,(uint16_t)(object->setSpeed/6.0f),0,(uint32_t)(-object->setPosition*(3200.0f*object->microStep/16*object->reduction_ratio)/360.0f),true,0);
		}
	}
}

//速度模式控制
void ZhangDaTou_Speedctr(ZDTMotor_Typedef* object,float SpeedVal)
{
	object->mod = 0;
	object->setSpeed = SpeedVal;
}	
//速度位置控制
void ZhangDaTou_PositionSpeedctr(ZDTMotor_Typedef* object,float SpeedVal,float PositionVal)
{
	object->mod = 1;
	object->setSpeed = SpeedVal;
	object->setPosition = PositionVal;
}
//获取速度
float ZhangDaTou_getSpeedDate(ZDTMotor_Typedef* object)
{
	return object->Speed;
}
//获取位置
float ZhangDaTou_getPositionDate(ZDTMotor_Typedef* object)
{
	return object->Position;
}
//设置变量所代表的电机id,
void ZhangDaTou_init(ZDTMotor_Typedef* object,uint8_t id)
{
	object->id = id;
	if (object->huart == NULL) {
		object->huart = &huart1;
	}
}
void ZhangDaTou_SetUart(ZDTMotor_Typedef* object, UART_HandleTypeDef *huart)
{
	object->huart = huart;
}
//任务函数
void ZhangDaTou_Task()
{	
	ZhangDaTou_Control(&pitchmotor);//控制电机1
	ZhangDaTou_Control(&yawmotor);//控制电机2
}

