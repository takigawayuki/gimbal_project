#include "common.h"

void camera_data_update(float dx, float dy)
{
    sys.value.camera_x = dx;
    sys.value.camera_y = dy;
}