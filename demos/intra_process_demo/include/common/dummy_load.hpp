#ifndef DEMOS__DUMMY_LOAD_H
#define DEMOS__DUMMY_LOAD_H

#include <stdint.h>
#include <stdlib.h>
#include "rclcpp/executors/exp_config.hpp"

void dummy_load_ms(int load_ms);
void dummy_load_100us(int load_100us);
void dummy_load_calibration();

#endif