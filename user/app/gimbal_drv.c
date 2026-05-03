#include "common.h"
#include "ZhangDaTou.h"

void period_init(void);
void pid_init(void);

void gimbal_init(void)
{
    period_init();
    pid_init();
}

void period_init(void)
{
    sys.period.sys_fs = 1000;   // 系统运行频率 1000Hz
    sys.period.sys_ts = 1.0f / sys.period.sys_fs; // 系统周期 = 1 / 系统频率 = 1ms

    //俯仰角pid运行频率初始化
    sys.period.camera_y_pid_fs = 1000;  // Y 轴 PID运行频率 1000Hz
    sys.period.camera_y_pid_ts = 1.0f / sys.period.camera_y_pid_fs; // PID周期 = 1 / PID频率 = 1ms
    sys.period.camera_y_pid_cnt_val = sys.period.sys_fs * sys.period.camera_y_pid_ts;  //每 1 次系统任务 → 跑 1 次 PID

    //偏航角pid运行频率初始化
    sys.period.camera_x_pid_fs = 1000;  // X 轴 PID运行频率 1000Hz
    sys.period.camera_x_pid_ts = 1.0f / sys.period.camera_x_pid_fs;
    sys.period.camera_x_pid_cnt_val = sys.period.sys_fs * sys.period.camera_x_pid_ts;

}

void pid_init(void)
{
    //pitch
    sys.camera_y_pid.kp = 0.0f;
    sys.camera_y_pid.ki = 0.0f;
    sys.camera_y_pid.kd = 0.0f;
    sys.camera_y_pid.out_max = 10.0f;
    sys.camera_y_pid.out_min = -10.0f;
    sys.camera_y_pid.i_term_max = 10.0f;
    sys.camera_y_pid.i_term_min = -10.0f;
    sys.camera_y_pid.ts = sys.period.camera_y_pid_ts;
    sys.camera_y_pid.i_isolate_flag = 0U;

    //yaw
    sys.camera_x_pid.kp = 0.0f;
    sys.camera_x_pid.ki = 0.0f;
    sys.camera_x_pid.kd = 0.0f;
    sys.camera_x_pid.out_max = 10.0f;
    sys.camera_x_pid.out_min = -10.0f;
    sys.camera_x_pid.i_term_max = 10.0f;
    sys.camera_x_pid.i_term_min = -10.0f;
    sys.camera_x_pid.ts = sys.period.camera_x_pid_ts;
    sys.camera_x_pid.i_isolate_flag = 0U;
}


void camera_y_pid_ctrl(sys_t *sys , float ref_value) 
{
   if(++sys->period.camera_y_pid_cnt >= sys->period.camera_y_pid_cnt_val)
   {
       sys->period.camera_y_pid_cnt = 0;
       parallel_pid_ctrl(&sys->camera_y_pid, ref_value, sys->value.camera_y);
   }

    //张大头电机控制
    float motor_speed = sys->camera_y_pid.out_value;
    ZhangDaTou_Speedctr(&pitchmotor, motor_speed, 1000);  
    ZhangDaTou_Control(&pitchmotor);
}


void camera_x_pid_ctrl(sys_t *sys , float ref_value) 
{
   if(++sys->period.camera_x_pid_cnt >= sys->period.camera_x_pid_cnt_val)
   {
       sys->period.camera_x_pid_cnt = 0;
       parallel_pid_ctrl(&sys->camera_x_pid, ref_value, sys->value.camera_x);
   }

    //张大头电机控制
    float motor_speed = sys->camera_x_pid.out_value;
    ZhangDaTou_Speedctr(&yawmotor, motor_speed, yawmotor.setAcc);
    ZhangDaTou_Control(&yawmotor);
}

