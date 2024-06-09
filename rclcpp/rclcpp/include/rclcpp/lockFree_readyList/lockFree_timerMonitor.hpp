#ifndef rclcpp_lockFree_timer_list
#define rclcpp_lockFree_timer_list

#include <atomic>
#include <vector>
#include <map>
#include <unordered_map>
#include <iostream>
#include "rclcpp/executors/exp_config.hpp"

using namespace std;

typedef struct{
    int64_t next_call_time;
    uintptr_t callback;
}callback_map;

extern int Timer_num;

// pointer to global timerList
extern atomic< callback_map* > ptr_global_timerList;
// entity of global timerList   
extern int map_num;
extern callback_map map_global_timerList[TIMER_MAX_NUM];
// structure used to store ptr to local timerList of each thread
extern unordered_map< uint32_t , callback_map *> TID_to_MAP;

extern int tid_num;
extern callback_map TID_MAP[TID_MAX_NUM][TIMER_MAX_NUM];

void timerList_init();

#endif