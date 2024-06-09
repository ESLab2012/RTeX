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
#define LOCAL_PUB_NUM         5           //MARK: [3/5/10]
#define LOCAL_THREAD_NUM      2           //MARK: thread number
#define LOCAL_WORKLOAD_EMPTY  0           //MARK: size of dummy workload
//----------------------------------------------------------------------------------
#define TIMER_Load_MS         LOCAL_WORKLOAD_EMPTY
#define FUSION_Load_MS        LOCAL_WORKLOAD_EMPTY
//----------------------------------------------------------------------------------
#define TIMER1_Period_MS      100ms 
#define TIMER2_Period_MS      100ms
#define TIMER3_Period_MS      100ms
#define TIMER4_Period_MS      100ms
#define TIMER5_Period_MS      100ms
#define TIMER6_Period_MS      100ms 
#define TIMER7_Period_MS      100ms
#define TIMER8_Period_MS      100ms
#define TIMER9_Period_MS      100ms
#define TIMER10_Period_MS     100ms
//----------------------------------------------------------------------------------


using namespace std::chrono_literals;
extern volatile int32_t cnter_val[50];

#if (LOCAL_PUB_NUM==3)
  extern volatile int32_t fusion3_cache_[3];
  extern volatile int32_t fusion3_round;
#elif (LOCAL_PUB_NUM==5)
  extern volatile int32_t fusion5_cache_[5];
  extern volatile int32_t fusion5_round;
#elif (LOCAL_PUB_NUM==10)
  extern volatile int32_t fusion10_cache_[10];
  extern volatile int32_t fusion10_round;
#endif


int main(int argc, char * argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  //---> Calibrate dummy task parameter
  dummy_load_calibration();

#if (LOCAL_SCREEN_PRINT == 1)
#if (LOCAL_PUB_NUM == 3)
  FILE *fp = freopen("results/PI/N_TO_1SUB/LF_3_TO_1SUB.log", "w", stdout);
#endif

#if (LOCAL_PUB_NUM == 5)
  FILE *fp = freopen("results/PI/N_TO_1SUB/LF_5_TO_1SUB.log", "w", stdout);
#endif
  if(NULL == fp){
    printf("failed~\r\n");
    return -1;
  }
#endif

  rclcpp::init(argc, argv);
  rclcpp::executors::RTeXExecutor executor(rclcpp::ExecutorOptions(), LOCAL_THREAD_NUM);

#if (LOCAL_PUB_NUM>=1)
  auto Timer1 = std::make_shared<Sensor>("Timer1", "dT1", 1, 1, TIMER_Load_MS, TIMER1_Period_MS);
  executor.add_node(Timer1);

#if (LOCAL_PUB_NUM>=3) 
  auto Timer2 = std::make_shared<Sensor>("Timer2", "dT2", 2, 1, TIMER_Load_MS, TIMER2_Period_MS);
  executor.add_node(Timer2);
  auto Timer3 = std::make_shared<Sensor>("Timer3", "dT3", 3, 1, TIMER_Load_MS, TIMER3_Period_MS);
  executor.add_node(Timer3);

#if (LOCAL_PUB_NUM>=5)  
  auto Timer4 = std::make_shared<Sensor>("Timer4", "dT4", 4, 1, TIMER_Load_MS, TIMER4_Period_MS);
  executor.add_node(Timer4);
  auto Timer5 = std::make_shared<Sensor>("Timer5", "dT5", 5, 1, TIMER_Load_MS, TIMER5_Period_MS);
  executor.add_node(Timer5);

#if (LOCAL_PUB_NUM>=10)
  auto Timer6 = std::make_shared<Sensor>("Timer6", "dT6", 6, 1, TIMER_Load_MS, TIMER6_Period_MS);
  executor.add_node(Timer6);
  auto Timer7 = std::make_shared<Sensor>("Timer7", "dT7", 7, 1, TIMER_Load_MS, TIMER7_Period_MS);
  executor.add_node(Timer7);
  auto Timer8 = std::make_shared<Sensor>("Timer8", "dT8", 8, 1, TIMER_Load_MS, TIMER8_Period_MS);
  executor.add_node(Timer8);
  auto Timer9 = std::make_shared<Sensor>("Timer9", "dT9", 9, 1, TIMER_Load_MS, TIMER9_Period_MS);
  executor.add_node(Timer9);
  auto Timer10 = std::make_shared<Sensor>("Timer10", "dT10", 10, 1, TIMER_Load_MS, TIMER10_Period_MS);
  executor.add_node(Timer10);
#endif
#endif
#endif
#endif


#if (LOCAL_PUB_NUM==3)
  auto Fusion = std::make_shared<Fusion3>("Fusion", "dT1", "dT2", "dT3", 1, 2, FUSION_Load_MS);
  executor.add_node(Fusion);
#elif (LOCAL_PUB_NUM==5)
  auto Fusion = std::make_shared<Fusion5>("Fusion", "dT1", "dT2", "dT3", "dT4", "dT5", 1, 2, FUSION_Load_MS);
  executor.add_node(Fusion);
#elif (LOCAL_PUB_NUM==10)
  auto Fusion = std::make_shared<Fusion10>("Fusion", "dT1", "dT2", "dT3", "dT4", "dT5", "dT6", "dT7", "dT8", "dT9", "dT10", 1, 2, FUSION_Load_MS);
  executor.add_node(Fusion);
#endif

  timerList_init();

  executor.spin();
  rclcpp::shutdown();

#if (LOCAL_SCREEN_PRINT == 1)
  fclose(fp);
#endif
}