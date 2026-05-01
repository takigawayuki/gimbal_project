#ifndef __COMMON_H
#define __COMMON_H

#include "main.h"
#include "string.h"
#include <stdbool.h>

/* ========== 数学运算宏 ========== */
#define SIGN(x)                   (((x) < 0.0f) ? -1.0f : 1.0f) // 返回符号（-1 或 1）
#define NORM2_f(x, y)             (sqrtf(SQ(x) + SQ(y)))         // 二维向量二范数

/* ========== 浮点异常处理宏 ========== */
#define UTILS_IS_INF(x)           ((x) == (1.0f / 0.0f) || (x) == (-1.0f / 0.0f)) // 判断无穷大
#define UTILS_IS_NAN(x)           ((x) != (x))                                        // 判断 NaN
#define UTILS_NAN_ZERO(x)         (x = UTILS_IS_NAN(x) ? 0.0f : x)                    // NaN 置零

#define MIN_MAX_LIMT(in, low, high)                                            \
  (in = in > high ? high : in < low ? low : in)
#define MAX_LIMT(in, outmax)                                                   \
  (in = in > outmax ? outmax : in < (-outmax) ? (-outmax) : in)

/* ========== 常用数学宏 ========== */
#define SQ(x)                     ((x) * (x)) // 平方
#define ABS(x)                    ((x) > 0 ? (x) : -(x)) // 绝对值
// #define MAX(x, y)     (((x) > (y)) ? (x) : (y))
// #define MIN(x, y)     (((x) < (y)) ? (x) : (y))
#define min(x, y)                 (((x) < (y)) ? (x) : (y)) // 返回较小值
#define max(x, y)                 (((x) > (y)) ? (x) : (y)) // 返回较大值
#define CLAMP(x, lower, upper)    (MIN(upper, MAX(x, lower))) // 限幅（依赖 MIN/MAX 定义）
#define FLOAT_EQU(floatA, floatB) ((ABS((floatA) - (floatB))) < 0.000001f) // 浮点近似相等比较

/* ========== 角度归一化宏 ========== */
#define wrap_pm_pi(theta)                                                      \
  theta = (theta > M_PI) ? theta - M_2PI : theta;                              \
  theta = (theta < -M_PI) ? theta + M_2PI : theta;
#define wrap_0_2pi(theta)                                                      \
  theta = (theta > M_2PI) ? theta - M_2PI : theta;                             \
  theta = (theta < 0.0f) ? theta + M_2PI : theta;
#define wrap_pm_2pi(theta)                                                      \
  theta = (theta > M_2PI) ? theta - M_2PI : theta;                              \
  theta = (theta < -M_2PI) ? theta + M_2PI : theta;

/* ========== 数学常量 ========== */
#define M_PI                    (3.14159265359f)          // π
#define M_2PI                   (6.28318530718f)          // 2π
#define M_2_PI                  (6.28318530718f)          // 2π（同义宏）
#define div_M_2PI               (0.159154943092391467f)   // 1/(2π)
#define SQRT3                   (1.73205080757f)          // √3
#define SQRT3_BY_2              (0.86602540378f)          // √3/2
#define ONE_BY_SQRT3            (0.57735026919f)          // 1/√3
#define TWO_BY_SQRT3            (1.15470053838f)          // 2/√3

/* ========== 位操作宏 ========== */
#define setbit(x, y)            (x |= (1 << y))           // 置位第 y 位
#define clrbit(x, y)            (x &= ~(1 << y))          // 清零第 y 位
#define reversebit(x, y)        (x ^= (1 << y))           // 翻转第 y 位
#define getbit(x, y)            ((x) >> (y) & 1)          // 读取第 y 位


/**
***********************************************************************
 * @brief 系统周期控制结构体
 * @note 包含启动计数器、测量计数器等多个计数器，以及对应的执行周期和频率参数
***********************************************************************
**/
typedef struct
{
    uint32_t sys_cnt;
    uint32_t camera_y_pid_cnt;
    uint32_t camera_x_pid_cnt;

    float sys_ts;
    float camera_y_pid_ts;
    float camera_x_pid_ts;

    uint8_t sys_ts_cnt_val; 
    uint8_t camera_y_pid_ts_cnt_val;
    uint8_t camera_x_pid_ts_cnt_val;

} period_t;

/**
***********************************************************************
 * @brief PID 控制器参数结构体
 * @note 
***********************************************************************
**/
typedef struct
{
    volatile float kp;          // 比例增益
    volatile float ki;          // 积分增益
    volatile float kd;          // 微分增益

    volatile float kfp;         // 前馈比例增益
    volatile float kf_damp;     // 前馈阻尼系数

    volatile float p_term;      // 比例项输出
    volatile float i_term;      // 积分项输出
    volatile uint8_t i_isolate_flag; // 积分清除隔离标志（1:清零并隔离积分, 0:正常积分）
    volatile float d_term;      // 微分项输出

    volatile float i_term_max;  // 积分项上限
    volatile float i_term_min;  // 积分项下限

    volatile float ts;          // 控制器采样周期

    volatile float ref_value;   // 参考输入值
    volatile float fback_value; // 反馈输入值

    volatile float error;       // 当前误差
    volatile float pre_err;     // 上一周期误差

    volatile float out_min;     // 输出下限
    volatile float out_max;     // 输出上限

    volatile float out_value;   // 控制器输出值
} pid_para_t;

typedef struct 
{
  float camera_x_pid;
  float camera_y;

  float gimbal_pitch;
  float gimbal_yaw;
}gimbal_value_t;

typedef struct 
{
  float pitch_set;
  float yaw_set;
  float camera_x_pid_set;
  float camera_y_set;
}gimbal_ctrl_t;

typedef struct
{
  period_t period;
  gimbal_value_t value;
  gimbal_ctrl_t ctrl;
  pid_para_t camera_y_pid;
  pid_para_t camera_x_pid;
  pid_para_t xxx_pid;
}sys_t;

extern sys_t sys;

/*** gimbal_calc.c ***/

/*** gimbal_drv.c ***/

/*** gimbal_ctrl.c ***/

/*** pid_drv.c ***/
float parallel_pid_ctrl(pid_para_t *pid, float ref_value, float fdback_value);


#endif
