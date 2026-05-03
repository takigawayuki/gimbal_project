#include "user_tim.h"
#include "common.h"

extern TIM_HandleTypeDef htim7;

void User_TIM_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim7);  // 启动 TIM7 中断
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        gimbal_task_state();  // 每 1ms 执行一次
    }
}