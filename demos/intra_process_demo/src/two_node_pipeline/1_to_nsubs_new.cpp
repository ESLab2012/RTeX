#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/lockFree_readyList/lockFree_readyList.hpp"
#include "rclcpp/lockFree_readyList/lockFree_timerMonitor.hpp"
#include "rclcpp/executors/exp_config.hpp"
#include <unistd.h>
#include "common/dummy_load.hpp"
#include "common/node_common.hpp"

/*=========================== For Experiments ======================================*/
//---> DEBUG-variables
#define LOCAL_SCREEN_PRINT    0           //MARK: 0-screen, 1-file
//---> EXP-variables
#define LOCAL_SUB_NUM         5           //MARK: [1/3/5/10]
#define LOCAL_THREAD_NUM      2           //MARK: thread number
#define LOCAL_WORKLOAD_EMPTY  0           //MARK: size of dummy workload
//--------------------------------
#define TIMER_Load_MS       LOCAL_WORKLOAD_EMPTY
#define TIMER_Period_MS     100ms 
//--------------------------------
#define SUB1_Load_MS        LOCAL_WORKLOAD_EMPTY
#define SUB2_Load_MS        LOCAL_WORKLOAD_EMPTY
#define SUB3_Load_MS        LOCAL_WORKLOAD_EMPTY
#define SUB4_Load_MS        LOCAL_WORKLOAD_EMPTY
#define SUB5_Load_MS        LOCAL_WORKLOAD_EMPTY
#define SUB6_Load_MS        LOCAL_WORKLOAD_EMPTY
#define SUB7_Load_MS        LOCAL_WORKLOAD_EMPTY
#define SUB8_Load_MS        LOCAL_WORKLOAD_EMPTY
#define SUB9_Load_MS        LOCAL_WORKLOAD_EMPTY
#define SUB10_Load_MS       LOCAL_WORKLOAD_EMPTY
//--------------------------------

using namespace std::chrono_literals;
extern volatile int32_t cnter_val[50];
//---> Calibrate dummy task parameter

int main(int argc, char * argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  dummy_load_calibration();

#if (LOCAL_SCREEN_PRINT == 1)
#if (LOCAL_SUB_NUM == 1)
  FILE *fp = freopen("results/1_TO_NSUBS/LF_1_to_1SUB.log", "w", stdout);
#endif
#if (LOCAL_SUB_NUM == 3)
  FILE *fp = freopen("results/1_TO_NSUBS/LF_1_to_3SUBS.log", "w", stdout);
#endif
#if (LOCAL_SUB_NUM == 5)
  FILE *fp = freopen("results/1_TO_NSUBS/LF_1_to_5SUBS.log", "w", stdout);
#endif
#if (LOCAL_SUB_NUM == 10)
  FILE *fp = freopen("results/1_TO_NSUBS/LF_1_to_10SUBS.log", "w", stdout);
#endif
  if(NULL == fp){
    printf("failed~\r\n");
    return -1;
  }
#endif

  rclcpp::init(argc, argv);
  rclcpp::executors::RTeXExecutor executor(rclcpp::ExecutorOptions(), LOCAL_THREAD_NUM);

#if (LOCAL_SUB_NUM==1) 
  auto Timer1 = std::make_shared<Sensor>("Timer1", "dummyTopic", 1, 1, TIMER_Load_MS, TIMER_Period_MS);
  executor.add_node(Timer1);
#endif
#if (LOCAL_SUB_NUM==3) 
  auto Timer1 = std::make_shared<Sensor>("Timer1", "dummyTopic", 3, 1, TIMER_Load_MS, TIMER_Period_MS);
  executor.add_node(Timer1);
#endif
#if (LOCAL_SUB_NUM==5) 
  auto Timer1 = std::make_shared<Sensor>("Timer1", "dummyTopic", 5, 1, TIMER_Load_MS, TIMER_Period_MS);
  executor.add_node(Timer1);
#endif
#if (LOCAL_SUB_NUM==10) 
  auto Timer1 = std::make_shared<Sensor>("Timer1", "dummyTopic", 10, 1, TIMER_Load_MS, TIMER_Period_MS);
  executor.add_node(Timer1);
#endif

#if (LOCAL_SUB_NUM>=1)
  auto Sub1 = std::make_shared<Command>("Sub1", "dummyTopic", 1, 2, SUB1_Load_MS);
  executor.add_node(Sub1);
#if (LOCAL_SUB_NUM>=3)
  auto Sub2 = std::make_shared<Command>("Sub2", "dummyTopic", 2, 3, SUB2_Load_MS);
  executor.add_node(Sub2);
  auto Sub3 = std::make_shared<Command>("Sub3", "dummyTopic", 3, 4, SUB3_Load_MS);
  executor.add_node(Sub3);
#if (LOCAL_SUB_NUM>=5)
  auto Sub4 = std::make_shared<Command>("Sub4", "dummyTopic", 4, 5, SUB4_Load_MS);
  executor.add_node(Sub4);
  auto Sub5 = std::make_shared<Command>("Sub5", "dummyTopic", 5, 6, SUB5_Load_MS);
  executor.add_node(Sub5);
#if (LOCAL_SUB_NUM>=10)
  auto Sub6 = std::make_shared<Command>("Sub6", "dummyTopic", 6, 7, SUB6_Load_MS);
  executor.add_node(Sub6);
  auto Sub7 = std::make_shared<Command>("Sub7", "dummyTopic", 7, 8, SUB7_Load_MS);
  executor.add_node(Sub7);
  auto Sub8 = std::make_shared<Command>("Sub8", "dummyTopic", 8, 9, SUB8_Load_MS);
  executor.add_node(Sub8);
  auto Sub9 = std::make_shared<Command>("Sub9", "dummyTopic", 9, 10, SUB9_Load_MS);
  executor.add_node(Sub9);
  auto Sub10 = std::make_shared<Command>("Sub10", "dummyTopic", 10, 11, SUB10_Load_MS);
  executor.add_node(Sub10);
    #endif
  #endif
#endif
#endif

  timerList_init();

  executor.spin();
  rclcpp::shutdown();

#if (LOCAL_SCREEN_PRINT == 1)
  fclose(fp);
#endif
  return 0;
}
