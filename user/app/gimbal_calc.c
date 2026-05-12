#include "common.h"

// extern volatile uint8_t target_valid;
extern volatile uint32_t target_lost_cnt;

void camera_data_update(float dx, float dy)
{
    sys.value.camera_x = dx;
    sys.value.camera_y = dy;

    // target_valid = 1; // 收到数据 → 有目标(不管 dx/dy 是否为 0)
    // target_lost_cnt = 0;

    if (dx != 0.0f || dy != 0.0f)
        target_lost_cnt = 0;
}
