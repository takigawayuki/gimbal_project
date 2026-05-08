#include "user_tim.h"
#include "common.h"

// #include "ZhangDaTou.h"

// int key_1 = 0;

extern TIM_HandleTypeDef htim7;

// #define KEY_ON 1
// #define KEY_OFF 0
// uint8_t key_scan(void);

void User_TIM_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim7); // 启动 TIM7 中断
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        // target_lost_cnt++;

        // gimbal_sm();
        gimbal_task_state();  // 每 1ms 执行一次

        // if (key_scan() == KEY_ON)
        // {
        //     key_1 = 1;
        // }
        // if (key_1 == 1)
        // {
        //     static int status;
        //     if (status == 0)
        //     {
        //         ZhangDaTou_Speedctr(&yawmotor, 15.0f, yawmotor.setAcc);
        //         ZhangDaTou_Control(&yawmotor);

        //         if (sys.value.camera_x != 0)
        //         {
        //             status = 1;
        //             // ZhangDaTou_Speedctr(&yawmotor,0.0f, yawmotor.setAcc);
        //             // ZhangDaTou_Control(&yawmotor);
        //         }
        //     }
        //     if (status == 1)
        //     {
        //         camera_x_pid_ctrl(&sys, 0.0f);
        //         camera_y_pid_ctrl(&sys, 0.0f);
        //         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        //     }
        // }
    }
}
