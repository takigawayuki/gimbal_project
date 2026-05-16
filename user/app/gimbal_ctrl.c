#include "common.h"
#include "ZhangDaTou.h"
#include "math.h"

// #define SEARCH_TIMEOUT 5000 // 假设找靶超时时间为5000ms

#define AIM_STABLE_CNT 150  // 1ms tick 计数，连续 150 次稳 → 开 
#define AIM_STABLE_FRAMES 6   // 连续 5 帧稳定才开激光

#define AIM_UNSTABLE_CNT 20 // 连续 20 次不稳才关
#define AIM_THRESHOLD 5.0f // 允许误差（像素）

// #define LASER_K 0.0f
// #define LASER_B 0.0f

#define LOST_BACK_TO_SEARCH_CNT 300 // 连续 300ms 丢靶 → 回搜索

sys_t sys;
// gimbal_sm_t gimbal_sm_obj = {GIMBAL_IDLE, 0, 0, 1};
gimbal_sm_t gimbal_sm_obj = {GIMBAL_IDLE, 0, 0};

volatile uint32_t target_lost_cnt = 0;
// volatile uint8_t target_valid = 0;
volatile uint8_t aim_stable_frames = 0;

static uint16_t stable_cnt = 0;
static uint16_t unstable_cnt = 0;
static uint8_t laser_on = 0;

/*** Key Scan ***/
// uint8_t key_1 = 0;
// #define KEY_ON 1
// #define KEY_OFF 0

// uint8_t key_scan(void)
// {
//     static uint8_t last = 1;
//     uint8_t cur = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

//     if (last == 1 && cur == 0)
//     {
//         last = 0;
//         return KEY_ON;
//     }

//     if (cur == 1)
//     {
//         last = 1;
//     }

//     return KEY_OFF;
// }

/*** target_found ***/
// uint8_t target_found(void)
// {
//     return target_valid;
// }

/*** target_stable ***/
uint8_t target_stable(void)
{
    // if (!target_found())
    //     return 0; // 没数据 → 不稳

    if (fabs(sys.value.camera_x) < AIM_THRESHOLD &&
        fabs(sys.value.camera_y) < AIM_THRESHOLD)
        return 1;
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

/*** 这个是经过实际测试能够用的 ***/
// void gimbal_task_state(void)
// {
//     if (key_scan() == KEY_ON)
//     {
//         key_1 = 1;
//     }
//     if (key_1 == 1)
//     {
//         static int status;
//         if (status == 0)
//         {
//             ZhangDaTou_Speedctr(&yawmotor, -15.0f, yawmotor.setAcc);
//             ZhangDaTou_Control(&yawmotor);

//             // if (target_found())
//             if (sys.value.camera_x != 0)
//             {
//                 status = 1;
//                 // ZhangDaTou_Speedctr(&yawmotor,0.0f, yawmotor.setAcc);
//                 // ZhangDaTou_Control(&yawmotor);
//             }
//         }
//         if (status == 1)
//         {
//             static uint8_t stable_cnt = 0;
//             static uint8_t unstable_cnt = 0;
//             static uint8_t laser_on = 0;
//             camera_x_pid_ctrl(&sys, 0.0f);
//             camera_y_pid_ctrl(&sys, 0.0f);

//             // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
//             if (target_stable())
//             {
//                 // stable_cnt++;
//                 // if (stable_cnt >= AIM_STABLE_CNT)
//                 // {
//                 //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);   // 开激光
//                 //     if (stable_cnt > AIM_STABLE_CNT) stable_cnt = AIM_STABLE_CNT;  // 防溢出
//                 // }

//                 unstable_cnt = 0;
//                 if (stable_cnt < AIM_STABLE_CNT)
//                     stable_cnt++;
//                 if (stable_cnt >= AIM_STABLE_CNT && !laser_on)
//                 {
//                     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
//                     laser_on = 1;
//                 }
//             }
//             else
//             {
//                 // stable_cnt = 0;                         // 任意一次不稳就清零
//                 // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);     // 关激光

//                 stable_cnt = 0;
//                 if (unstable_cnt < AIM_UNSTABLE_CNT)
//                     unstable_cnt++;
//                 if (unstable_cnt >= AIM_UNSTABLE_CNT && laser_on)
//                 {
//                     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
//                     laser_on = 0;
//                 }
//             }
//         }
//     }
// }

// void gimbal_task_state(void)
// {
//     if (key_scan() == KEY_ON)
//     {
//         key_1 = 1;
//     }
//     if (key_1 == 1)
//     {
//         static int status = 0;
//         static uint8_t stable_cnt = 0;
//         static uint8_t unstable_cnt = 0;
//         static uint8_t laser_on = 0;
//         static uint16_t lost_cnt = 0;

//         if (status == 0)                           // ── 搜索 ──
//         {
//             ZhangDaTou_Speedctr(&yawmotor, 15.0f, yawmotor.setAcc);
//             ZhangDaTou_Control(&yawmotor);
//             if (laser_on)                         // 如果激光开的就给他关掉
//             {
//                 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
//                 laser_on = 0;
//             }
//             stable_cnt = 0;
//             unstable_cnt = 0;

//             // ZhangDaTou_Speedctr(&yawmotor, 15.0f, yawmotor.setAcc);
//             // ZhangDaTou_Control(&yawmotor);

//             if (target_found())     // 搜到靶了，停止，置status=1
//             {
//                 // ZhangDaTou_Speedctr(&yawmotor, 0.0f, yawmotor.setAcc);
//                 // ZhangDaTou_Control(&yawmotor);

//                 camera_x_pid_ctrl(&sys, 0.0f);
//                 camera_y_pid_ctrl(&sys, 0.0f);
//                 lost_cnt = 0;
//                 status = 1;
//             }
//         }
// //         else                                       // ── 跟踪 ──
// //         {
// //             if (!target_found())    // 没有找到，重新回去找
// //             {
// //                 lost_cnt++;
// //                 if (lost_cnt >= LOST_BACK_TO_SEARCH_CNT)
// //                 {
// //                     lost_cnt = 0;
// //                     status = 0;
// //                     return;
// //                 }
// //             }
// //             else
// //             {
// //                 lost_cnt = 0;
// //             }

// //             camera_x_pid_ctrl(&sys, 0.0f);
// //             camera_y_pid_ctrl(&sys, 0.0f);

// //             if (target_stable())    // 偏差为0 → 稳定
// //             {
// //                 unstable_cnt = 0;
// //                 if (stable_cnt < AIM_STABLE_CNT) stable_cnt++;
// //                 if (stable_cnt >= AIM_STABLE_CNT && !laser_on)
// //                 {
// //                     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
// //                     laser_on = 1;
// //                 }
// //             }
// //             else
// //             {
// //                 stable_cnt = 0;
// //                 if (unstable_cnt < AIM_UNSTABLE_CNT) unstable_cnt++;
// //                 if (unstable_cnt >= AIM_UNSTABLE_CNT && laser_on)
// //                 {
// //                     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
// //                     laser_on = 0;
// //                 }
// //             }
// //         }
// //     }

//     }

// }

void gimbal_task_state(void)
{
    switch (gimbal_sm_obj.state)
    {
    case GIMBAL_IDLE:
        // 没有转移（转移全由菜单触发）
        // 动作：停电机 + 关激光。每 tick 都踩一次，保证干净
        ZhangDaTou_Speedctr(&yawmotor, 0.0f, yawmotor.setAcc);
        ZhangDaTou_Control(&yawmotor);
        ZhangDaTou_Speedctr(&pitchmotor, 0.0f, pitchmotor.setAcc);
        ZhangDaTou_Control(&pitchmotor);
        HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
        stable_cnt = 0;
        unstable_cnt = 0;
        laser_on = 0;
        break;

    case GIMBAL_SEARCH_LEFT:
        // 转移：看到目标 → TRACK
        if (sys.value.camera_x != 0)
        {
            // gimbal_sm_obj.state = GIMBAL_TRACK;
            // 从这里分流
            // if (menu.cur_item == MENU_ITEM_TRACK_STATIC)
            //     gimbal_sm_obj.state = GIMBAL_STATIC_TRACK;
            // else
            //     gimbal_sm_obj.state = GIMBAL_DYNAMIC_TRACK;

            // if (menu.cur_item == MENU_ITEM_TRACK_DYNAMIC)
            //     gimbal_sm_obj.state = GIMBAL_DYNAMIC_TRACK;
            // else
            //     gimbal_sm_obj.state = GIMBAL_STATIC_TRACK;

            gimbal_sm_obj.state = GIMBAL_STATIC_TRACK;

            break; // 这一 tick 不执行 SEARCH 动作，下 tick 走 TRACK
        }
        // 动作：匀速扫描 + 激光关
        // ZhangDaTou_Speedctr(&yawmotor,
        //                     80.0f * gimbal_sm_obj.scan_dir,
        //                     2000);
        ZhangDaTou_Speedctr(&yawmotor, -80.0f, 2000);
        ZhangDaTou_Control(&yawmotor);
        HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
        break;

    case GIMBAL_SEARCH_RIGHT:
        // 转移：看到目标 → TRACK
        if (sys.value.camera_x != 0)
        {
            // gimbal_sm_obj.state = GIMBAL_TRACK;
            // 从这里分流
            // if (menu.cur_item == MENU_ITEM_TRACK_STATIC)
            //     gimbal_sm_obj.state = GIMBAL_STATIC_TRACK;
            // else
            //     gimbal_sm_obj.state = GIMBAL_DYNAMIC_TRACK;

            // if (menu.cur_item == MENU_ITEM_TRACK_DYNAMIC)
            //     gimbal_sm_obj.state = GIMBAL_DYNAMIC_TRACK;
            // else
            //     gimbal_sm_obj.state = GIMBAL_STATIC_TRACK;

            gimbal_sm_obj.state = GIMBAL_STATIC_TRACK;

            break; // 这一 tick 不执行 SEARCH 动作，下 tick 走 TRACK
        }
        // 动作：匀速扫描 + 激光关
        // ZhangDaTou_Speedctr(&yawmotor,
        //                     80.0f * gimbal_sm_obj.scan_dir,
        //                     2000);
        ZhangDaTou_Speedctr(&yawmotor, +80.0f, 2000);
        ZhangDaTou_Control(&yawmotor);
        HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
        break;

    case GIMBAL_STATIC_TRACK:
        // 转移：丢失超过阈值 → SEARCH
        // if (sys.value.camera_x == 0 && sys.value.camera_y == 0)
        // {
        //     if (target_lost_cnt >= LOST_BACK_TO_SEARCH_CNT)  // 如果target_lost_cnt等于零，证明有摄像头有数据，如果不等于零，就证明没有数据，那就跳回到扫靶状态
        //     {
        //         // gimbal_sm_obj.state = GIMBAL_SEARCH_LEFT;

        //         if (menu.cur_item == MENU_ITEM_TRACK_STATIC_LEFT)
        //             gimbal_sm_obj.state = GIMBAL_SEARCH_LEFT;
        //         else
        //             gimbal_sm_obj.state = GIMBAL_SEARCH_RIGHT;

        //         HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
        //         stable_cnt = 0;
        //         unstable_cnt = 0;
        //         laser_on = 0;
        //         break;
        //     }
        // }
        // else
        // {
        //     target_lost_cnt = 0; // 有目标就清零
        // }
        // 动作：PID + 激光
        camera_x_pid_ctrl(&sys, 0.0f);
        camera_y_pid_ctrl(&sys, 0.0f);

        // 激光控制：稳定够久才开，抖动够久才关
        if (target_stable())
        {
            unstable_cnt = 0;
            // if (stable_cnt < AIM_STABLE_CNT)
            //     stable_cnt++;
            // if (stable_cnt >= AIM_STABLE_CNT && !laser_on)
            // {
            //     HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);
            //     laser_on = 1;
            // }

            if (aim_stable_frames >= AIM_STABLE_FRAMES && !laser_on)
            {
                HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);
                laser_on = 1;
            }

        }
        else
        {
            stable_cnt = 0;
            // if (unstable_cnt < AIM_UNSTABLE_CNT)
            //     unstable_cnt++;
            // if (unstable_cnt >= AIM_UNSTABLE_CNT && laser_on)
            // {
            //     HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
            //     laser_on = 0;
            // }
        }

        // if(laser_on == 1 && menu.cur_item == MENU_ITEM_TRACK_STATIC)
        if (laser_on == 1)
        {
            ZhangDaTou_Speedctr(&yawmotor, 0.0f, yawmotor.setAcc);
            ZhangDaTou_Control(&yawmotor);
            ZhangDaTou_Speedctr(&pitchmotor, 0.0f, pitchmotor.setAcc);
            ZhangDaTou_Control(&pitchmotor);
        }
        // else
        // {
        //     // 其他情况（动态模式 或 未瞄准稳定）→ 持续 PID
        //     camera_x_pid_ctrl(&sys, 0.0f);
        //     camera_y_pid_ctrl(&sys, 0.0f);
        // }
        break;

    case GIMBAL_DYNAMIC_TRACK:
        // 转移：丢失超过阈值 → SEARCH
        // if (sys.value.camera_x == 0 && sys.value.camera_y == 0)
        // {
        //     if (target_lost_cnt >= LOST_BACK_TO_SEARCH_CNT)
        //     {
        //         gimbal_sm_obj.state = GIMBAL_SEARCH;
        //         HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
        //         stable_cnt = 0;
        //         unstable_cnt = 0;
        //         laser_on = 0;
        //         break;
        //     }
        // }
        // else
        // {
        //     target_lost_cnt = 0; // 有目标就清零
        // }

        // 动作：PID + 激光
        camera_x_pid_ctrl(&sys, 0.0f);
        camera_y_pid_ctrl(&sys, 0.0f);

        // 激光控制：稳定够久才开，抖动够久才关
        if (target_stable())
        {
            unstable_cnt = 0;
            if (stable_cnt < AIM_STABLE_CNT)
                stable_cnt++;
            if (stable_cnt >= AIM_STABLE_CNT && !laser_on)
            {
                HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);
                laser_on = 1;
            }
        }
        else
        {
            stable_cnt = 0;
            // if (unstable_cnt < AIM_UNSTABLE_CNT)
            //     unstable_cnt++;
            // if (unstable_cnt >= AIM_UNSTABLE_CNT && laser_on)
            // {
            //     HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
            //     laser_on = 0;
            // }
        }
        break;

        // if(laser_on == 1 && menu.cur_item == MENU_ITEM_TRACK_STATIC)
        // if(laser_on == 1)
        // {
        //     ZhangDaTou_Speedctr(&yawmotor, 0.0f, yawmotor.setAcc);
        //     ZhangDaTou_Control(&yawmotor);
        //     ZhangDaTou_Speedctr(&pitchmotor, 0.0f, pitchmotor.setAcc);
        //     ZhangDaTou_Control(&pitchmotor);
        // }
        // else
        // {
        //     // 其他情况（动态模式 或 未瞄准稳定）→ 持续 PID
        //     camera_x_pid_ctrl(&sys, 0.0f);
        //     camera_y_pid_ctrl(&sys, 0.0f);
        // }

    default:
        break;
    }
}
