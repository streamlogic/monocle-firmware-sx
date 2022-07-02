#include "nrf_log.h"

// Include Moncole API definitions
#include "monocle.h"


////////// Forward declarations

void demo1();
void demo2();
void demo3();
void log_grayscale(uint8_t *buf, uint16_t buflen, uint16_t imgwidth);


////////// Main

int main(void) {
  //// Uncomment one of the demo apps:

  demo1();

  //demo2();

  //demo3();
}


////////// demo 1

// Simple pass-through display (color or grayscale)
// Works with StreamLogic monocle-minimal example
//
// UI:
//   * live display at boot
//   * any tap gesture to stop/start live display

void demo1() {
  bool on;

  // Required initialization
  monocole_boot(true);

  on = true;
  while (true) {
    // Wait for any touch gesture (blocks until gesture event)
    if (monocle_eventloop(EVT_GESTURE_ANY) != 0) {
        if (on) {
          monocle_camera_off();
          monocle_display_off();
        } else {
          monocle_display_on();
          monocle_camera_on();
        }
        on = !on;
    }
  }
}


////////// demo 2

// Instant replay demo
// Works with StreamLogic monocle-minimal example
//
// UI:
//   * display off at boot with background recording
//   * any tap gesture to replay past 4-8s

void demo2() {
  // Required initialization
  monocole_boot(true);
  monocle_display_off();

  while (true) {
    // Wait for any touch gesture (blocks until gesture event)
    if (monocle_eventloop(EVT_GESTURE_ANY) != 0) {
        // Stop camera and replay
        monocle_camera_off();
        monocle_display_on();
        monocle_buffer_replay(4);

        // Use timeout to wait (do not use nrf_delay)
        monocle_app_timer(10000);
        monocle_eventloop(EVT_APP_TIMEOUT_F);

        // Back to record mode
        monocle_buffer_resume();
        monocle_display_off();
        monocle_camera_on();
    }
  }
}


////////// demo 3

// Photo capture
// Works with StreamLogic monocle-readout example
//
// UI:
//   * live display at boot
//   * any tap gesture to capture photo and stop display
//   * any tap gesture to restart

void demo3() {
  bool on;
  uint8_t *readbuf;
  uint16_t buflen;

  // Required initialization
  monocole_boot(true);

  on = true;
  while (true) {
    // Wait for any touch gesture (blocks until gesture event)
    if (monocle_eventloop(EVT_GESTURE_ANY) != 0) {
      // Read out image from frame buffer
      monocle_capture();
      while (readbuf = monocle_read_capture(&buflen)) {
        // Utility to display gray image as ASCII to log
        // (Note: image comes out from bottom to top)
        log_grayscale(readbuf, buflen, 640);
      }
    }

    monocle_camera_off();
    monocle_display_off();
    // Wait for any touch gesture (blocks until gesture event)
    monocle_eventloop(EVT_GESTURE_ANY);
    monocle_display_on();
    monocle_camera_on();
  }
}


// Generate ASCII as data arrives and output when reach end of line

static char graymap[10] = "@%#*+=-:. ";

void log_grayscale(uint8_t *buf, uint16_t buflen, uint16_t imgwidth) {
  // Pixel averaging (downsize by 10)
  static uint16_t acc = 0;
  static uint8_t pxl_cnt = 0;
  static uint8_t line_cnt = 0;

  // Output line
  static int line_pos = 0;
  static char linebuf[129];

  for (uint8_t i=0; i<buflen; i++) {
    acc += buf[i];
    pxl_cnt++;
    if (pxl_cnt == 10) {
      int lvl = acc>>8;
      sprintf(linebuf+line_pos, "%c%c", graymap[lvl], graymap[lvl]);
      line_pos += 2;

      acc = 0;
      pxl_cnt = 0;

      // Handle end of line
      if (line_pos == 128) {
        line_cnt += 1;
        if (line_cnt == 10) {
          linebuf[line_pos] = 0;
          NRF_LOG_INFO("%s",NRF_LOG_PUSH(linebuf));
          line_cnt = 0;
        }
        line_pos = 0;
      }
    }
  }
}
