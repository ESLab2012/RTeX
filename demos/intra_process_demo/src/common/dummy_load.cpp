#include "common/dummy_load.hpp"

#define DUMMY_LOAD_ITER	      100

int dummy_load_calib = 1;

void dummy_load_ms(int load_ms) {
  int i, j;
  for (j = 0; j < dummy_load_calib * load_ms; j++)
      for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
          __asm__ volatile ("nop");
}
void dummy_load_100us(int load_100us) {
  int i, j;
  for (j = 0; j < (dummy_load_calib * load_100us /10); j++)
      for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
          __asm__ volatile ("nop");
}

void dummy_load_calibration() {
  volatile uint64_t ts_start, ts_end;
  while(1) {
    ts_start = get_clocktime(); // in ns
    dummy_load_ms(100);         // 100ms
    ts_end = get_clocktime();   // in ns
    int duration_ns = ts_end - ts_start;
    if (abs(duration_ns - 100*1000*1000) < 1000000) {// error margin: 1ms
      break;
    }
    dummy_load_calib = 100*1000*1000*dummy_load_calib / duration_ns;
    if (dummy_load_calib <= 0) {
      dummy_load_calib = 1;
    }
  }
  ts_start = get_clocktime(); // in ns
  dummy_load_ms(10);          // 10ms
  ts_end = get_clocktime();   // in ns
  printf("|CALIBRATION TEST|[Setting: 10ms]->@time-measure: %lu. \r\n", ts_end-ts_start);
}