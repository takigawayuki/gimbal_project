#include "common.h"

menu_t menu;

extern volatile uint32_t target_lost_cnt;
// extern volatile uint8_t  target_valid;
extern gimbal_sm_t gimbal_sm_obj;

void menu_init(void)
{
    menu.cur_item = MENU_ITEM_STANDBY;
    menu.in_running = 0;
}

void menu_update(key_event_t ev_menu, key_event_t ev_enter)
{
    if (menu.in_running == 0)
    {
        // ===== ≤Àµ•—°‘ÒÃ¨ =====
        if (ev_menu == KEY_EVENT_SHORT)
        {
            menu.cur_item = (menu.cur_item + 1) % MENU_ITEM_COUNT;
        }
        if (ev_enter == KEY_EVENT_SHORT)
        {
            menu.in_running = 1;
            switch (menu.cur_item)
            {
                case MENU_ITEM_STANDBY:
                    gimbal_sm_obj.state = GIMBAL_IDLE;
                    break;

                // case MENU_ITEM_TRACK_STATIC:
                case MENU_ITEM_TRACK_STATIC_LEFT:
                    // gimbal_sm_obj.scan_dir = -1;          // Õ˘◊Û…®
                    // gimbal_sm_obj.state = GIMBAL_SEARCH;
                    gimbal_sm_obj.state = GIMBAL_SEARCH_LEFT;
                    break;

                case MENU_ITEM_TRACK_STATIC_RIGHT:
                    // gimbal_sm_obj.scan_dir = 1;           // Õ˘”“…®
                    // gimbal_sm_obj.state = GIMBAL_SEARCH;
                    gimbal_sm_obj.state = GIMBAL_SEARCH_RIGHT;
                    break;

                case MENU_ITEM_TRACK_DYNAMIC:
                    // gimbal_sm_obj.state = GIMBAL_SEARCH;
                    // gimbal_sm_obj.scan_dir = -1; 
                    gimbal_sm_obj.state = GIMBAL_DYNAMIC_TRACK;
                    break;

                default:
                    break;
            }
        }
    }
    else
    {
        // ===== ‘À––Ã¨ =====
        if (ev_enter == KEY_EVENT_SHORT)
        {
            menu.in_running = 0;
            gimbal_sm_obj.state = GIMBAL_IDLE;
        }
        // PA4 ≤ªœÏ”¶
    }
}
