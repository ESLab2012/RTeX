#ifndef RCLCPP__EXECUTORS__EXP_CONFIG_HPP_
#define RCLCPP__EXECUTORS__EXP_CONFIG_HPP_

#include <stdint.h>
#include <stdio.h>
#include <time.h>

/*=========================== RTeX Configuration ============================*/
#define TID_MAX_NUM         5       // Max. number of thread/core in an Executor
#define TIMER_MAX_NUM       6       // Max. number of timers
#define CHAIN_NUM_MAX       15      // Max. number of chains, since chain index starts at 1, CHAIN_NUM_MAX must be at least 1 grater than actual number of chains.
#define CHAIN_NODE_MAX      5       // Max. number of nodes for a specific chain, since chain index starts at 2, CHAIN_NODE_MAX must be at least 2 grater than actual number of chains.
#define CSTACK_MAX_NUM      CHAIN_NUM_MAX + 10      // Max. number of pre-allocated `cStack`
#define POSTERFLAG_SIZE     10      // number of bits used for pointer marking
#define THREAD_MAX_NUM      TID_MAX_NUM


static inline uint64_t get_clocktime() { 
    long int        ns; 
    uint64_t        all; 
    time_t          sec; 
    struct timespec spec; 

    clock_gettime(CLOCK_REALTIME, &spec);

    sec   = spec.tv_sec; 
    ns    = spec.tv_nsec; 
    all   = (uint64_t) sec * 1000000000UL + (uint64_t) ns; 
    return all;  
}
#endif
