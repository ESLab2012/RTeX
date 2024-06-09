#include "rclcpp/lockFree_readyList/lockFree_readyList.hpp"
#include "rclcpp/lockFree_readyList/lockFree_timerMonitor.hpp"
#include <algorithm>

int Timer_num = 0;
atomic< callback_map* > ptr_global_timerList;

int map_num = 0;
callback_map map_global_timerList[TIMER_MAX_NUM];
unordered_map< uint32_t , callback_map *> TID_to_MAP;

int tid_num = 0;
callback_map TID_MAP[TID_MAX_NUM][TIMER_MAX_NUM];

void timerList_init() {
    callback_map* ptr_temp = ptr_global_timerList.load();
    ptr_global_timerList.compare_exchange_strong(ptr_temp, map_global_timerList);
    sort(ptr_global_timerList.load(),ptr_global_timerList.load()+Timer_num,[](callback_map a,callback_map b){
        return a.next_call_time < b.next_call_time;
    });
}