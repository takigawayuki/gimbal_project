#include "common.h"
#include "ZhangDaTou.h"
#include "math.h"

#define SEARCH_TIMEOUT 5000     // 假设找靶超时时间为5000ms

#define AIM_STABLE_CNT  30      // 允许误差连续30次认为目标稳定
#define AIM_THRESHOLD 5.0f      // 允许误差（像素 or 角度）

#define LASER_K  0.0f
#define LASER_B  0.0f

sys_t sys;
gimbal_sm_t gimbal_sm_obj = {GIMBAL_IDLE, 0, 0, 1};
volatile uint32_t target_lost_cnt = 50;
volatile uint8_t target_valid = 0;  // 1=有目标, 0=无目标


/*** Key Scan ***/
uint8_t key_1 = 0;  
#define KEY_ON  1
#define KEY_OFF 0

uint8_t key_scan(void)
{
    static uint8_t last = 1;
    uint8_t cur = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

    if (last == 1 && cur == 0)
    {
        last = 0;
        return KEY_ON;
    }

    if (cur == 1)
    {
        last = 1;
    }

    return KEY_OFF;
}

/*** target_found ***/
uint8_t target_found(void) 
{
    return target_valid;
}

/*** target_stable ***/
uint8_t target_stable(void)
{
    if (fabs(sys.value.camera_x) < AIM_THRESHOLD &&
        fabs(sys.value.camera_y) < AIM_THRESHOLD)
    {
        return 1;
    }
    return 0;
}

// /*** 切换状态 ***/
// void gimbal_sm(void)
// {
//     switch (gimbal_sm_obj.state)
//     {
//         case GIMBAL_IDLE:
//             if (key_scan() == KEY_ON)
//             {	
//                 gimbal_sm_obj.state = GIMBAL_SEARCH;
//             }
//             break;
        
//         case GIMBAL_SEARCH:
//            gimbal_sm_obj.search_timeout_cnt++;
//             if (target_found())
//             {
//                 gimbal_sm_obj.search_timeout_cnt = 0;
//                 gimbal_sm_obj.state = GIMBAL_TRACK;		
//             }
//             else if (gimbal_sm_obj.search_timeout_cnt >= SEARCH_TIMEOUT)
//             {
//                 gimbal_sm_obj.search_timeout_cnt = 0;
//                gimbal_sm_obj.state = GIMBAL_IDLE; // 超时回待机
//             }
//             break;

//        case GIMBAL_TRACK:
//            if (!target_found())
//            {
//                gimbal_sm_obj.aim_stable_cnt = 0;
//                gimbal_sm_obj.state = GIMBAL_SEARCH;
//            }
//            else if (target_stable())
//            {
//                gimbal_sm_obj.aim_stable_cnt++;
//                if (gimbal_sm_obj.aim_stable_cnt >= AIM_STABLE_CNT)
//                {
//                 //    gimbal_sm_obj.state = GIMBAL_LOCK;
//                }
//            }
//            else
//            {
//                gimbal_sm_obj.aim_stable_cnt = 0;
//            }
//            break;

//     //    case GIMBAL_LOCK:
//     //        if (!target_found())
//     //        {
//     //            gimbal_sm_obj.aim_stable_cnt = 0;
//     //            gimbal_sm_obj.state = GIMBAL_SEARCH;
//     //        }
//     //        break;

//     }


    
// }

// /*** 实际控制 ***/
// void gimbal_task_state(void)    // 中断里面运行
// {
//     // 接收两个轴电机的角度位置
//     float yaw_pos   = ZhangDaTou_getPositionDate(&yawmotor);
//     float pitch_pos = ZhangDaTou_getPositionDate(&pitchmotor);

//     // camera_x_pid_ctrl(&sys, 0.0f);
//     // camera_y_pid_ctrl(&sys, 0.0f);

//     switch (gimbal_sm_obj.state)
//     {
//         case GIMBAL_IDLE:
//             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
//             break;

//         case GIMBAL_SEARCH:
//             // 云台扫描靶子
//             ZhangDaTou_Speedctr(&yawmotor, 8.0f, yawmotor.setAcc);
//             ZhangDaTou_Control(&yawmotor);
//             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
//             break;

//         case GIMBAL_TRACK:
//             camera_x_pid_ctrl(&sys, 0.0f);
//             if(sys.camera_x_pid.out_value > 0)
//                 gimbal_sm_obj.scan_dir =  1;
//             else
//                 gimbal_sm_obj.scan_dir =  -1;	
//             camera_y_pid_ctrl(&sys, 0.0f);
//             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
//             break;

//     //    case GIMBAL_LOCK:
//     //    {
//     //        float offset_y = LASER_K * sys.value.distance + LASER_B;
//     //        camera_x_pid_ctrl(&sys, 0.0f);
//     //        camera_y_pid_ctrl(&sys, offset_y);
//     //        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
//     //        break;
//     //    }

//     }



// }


void gimbal_task_state(void)
{
    if (key_scan() == KEY_ON)
    {
        key_1 = 1;
    }
    if (key_1 == 1)
    {
        static int status;
        if (status == 0)
        {
            ZhangDaTou_Speedctr(&yawmotor, 15.0f, yawmotor.setAcc);
            ZhangDaTou_Control(&yawmotor);

            if (sys.value.camera_x != 0)
            {
                status = 1;
                // ZhangDaTou_Speedctr(&yawmotor,0.0f, yawmotor.setAcc);
                // ZhangDaTou_Control(&yawmotor);
            }
        }
        if (status == 1)
        {
            camera_x_pid_ctrl(&sys, 0.0f);
            camera_y_pid_ctrl(&sys, 0.0f);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        }
    }

}



