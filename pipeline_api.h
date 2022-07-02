#define FBREADOUT_ADDR 0x50
#define FBREADOUT_GET_STATUS8 0x00
#define FBREADOUT_READOUT_REQ 0x01
#define FBREADOUT_IMAGE_STRM 0x02
#define FRAMEBUF_ADDR 0x20
#define FRAMEBUF_GET_STATUS8 0x00
#define FRAMEBUF_GET_OLDEST32 0x01
#define FRAMEBUF_GET_NEWEST32 0x02
#define OVCAM_ADDR 0x10
#define OVCAM_GET_STATUS8 0x10
#define OVCAM_CAPTURE_REQ 0x00
#define OVCAM_START_REQ 0x01
#define OVCAM_PAUSE_REQ 0x02
#define OVCAM_XCLK_OFF_REQ 0x08
#define OVCAM_XCLK_ON_REQ 0x09
#define FBOUT_ADDR 0x30
#define FBOUT_GET_STATUS8 0x00
#define FBOUT_SET_SPEED8 0x08
#define FBOUT_SET_START32 0x09
#define FBOUT_STOP_REQ 0x01
#define FBOUT_PLAY_REQ 0x02
#define FBOUT_PAUSE_REQ 0x03
#define FBOUT_REPLAY_REQ 0x04
