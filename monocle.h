#include <stdbool.h>
#include <stdint.h>

#define EVT_LOW_BATTERY_F         0x0001
#define EVT_APP_TIMEOUT_F         0x0002
#define EVT_GESTURE_TAP_F         0x0010
#define EVT_GESTURE_DOUBLETAP_F   0x0020
#define EVT_GESTURE_PRESS_F       0x0040
#define EVT_GESTURE_LONGPRESS_F   0x0080
#define EVT_GESTURE_LONGBOTH_F    0x0100
#define EVT_GESTURE_ANY           0x01f0

extern bool monocole_boot(bool live_mode);
extern uint32_t monocle_eventloop(uint32_t event_mask);
extern void monocle_app_timer(uint32_t ms);

extern void monocle_camera_on();
extern void monocle_camera_off();
extern void monocle_buffer_replay(uint8_t speed);
extern void monocle_buffer_resume();
extern void monocle_capture();
extern uint8_t *monocle_read_capture(uint16_t *len_p);

extern void monocle_display_on();
extern void monocle_display_off();
