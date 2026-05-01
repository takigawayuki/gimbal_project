#include "common.h"

void period_init(void);
void pid_init(void);

void gimbal_init(void)
{
    period_init();
}

void period_init(void)
{
    sys.period.sys_fs = 1000;
    sys.period.sys_ts = 1.0f / sys.period.sys_fs;

    //俯仰角pid运行频率初始化
    sys.period.camera_y_pid_fs = 1000;
    sys.period.camera_y_pid_ts = 1.0f / sys.period.camera_y_pid_fs;
    sys.period.camera_y_pid_cnt_val = sys.period.sys_fs * sys.period.camera_y_pid_ts;

    //偏航角pid运行频率初始化
    sys.period.camera_x_pid_fs = 1000;
    sys.period.camera_x_pid_ts = 1.0f / sys.period.camera_x_pid_fs;
    sys.period.camera_x_pid_cnt_val = sys.period.sys_fs * sys.period.camera_x_pid_ts;

}

void pid_init(void)
{
    sys.camera_y_pid.kp = 0.0f;
    sys.camera_y_pid.ki = 0.0f;
    sys.camera_y_pid.kd = 0.0f;
    sys.camera_y_pid.out_max = 0.0f;
    sys.camera_y_pid.out_min = 0.0f;
    sys.camera_y_pid.i_term_max = 0.0f;
    sys.camera_y_pid.i_term_min = 0.0f;
    sys.camera_y_pid.ts = sys.period.camera_y_pid_ts;
    sys.camera_y_pid.i_isolate_flag = 0U;

    sys.camera_x_pid.kp = 0.0f;
    sys.camera_x_pid.ki = 0.0f;
    sys.camera_x_pid.kd = 0.0f;
    sys.camera_x_pid.out_max = 0.0f;
    sys.camera_x_pid.out_min = 0.0f;
    sys.camera_x_pid.i_term_max = 0.0f;
    sys.camera_x_pid.i_term_min = 0.0f;
    sys.camera_x_pid.ts = sys.period.camera_x_pid_ts;
    sys.camera_x_pid.i_isolate_flag = 0U;
}

void camera_y_pid_ctrl(sys_t *sys , float ref_value)
{
    if(++sys->period.camera_y_pid_cnt >= sys->period.camera_y_pid_cnt_val)
    {
        sys->period.camera_y_pid_cnt = 0;
        parallel_pid_ctrl(&sys->camera_y_pid, ref_value, sys->value.camera_y_pid);
    }

    //张大头电机控制
}

void camera_x_pid_ctrl(sys_t *sys , float ref_value)
{
    if(++sys->period.camera_x_pid_cnt >= sys->period.camera_x_pid_cnt_val)
    {
        sys->period.camera_x_pid_cnt = 0;
        parallel_pid_ctrl(&sys->camera_x_pid, ref_value, sys->value.camera_x_pid);
    }

    //张大头电机控制
}

