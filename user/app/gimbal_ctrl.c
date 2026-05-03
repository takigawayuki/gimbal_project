#include "common.h"

sys_t sys;

void gimbal_task_state(void)
{
    
    camera_x_pid_ctrl(&sys, 0.0f);
    camera_y_pid_ctrl(&sys, 0.0f); 
	
}

