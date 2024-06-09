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
#include "common/node_common.hpp"
#include "common/dummy_load.hpp"
#include "rmw/rmw.h"

/*=========================== For Experiments ======================================*/

#define LOCAL_SCREEN_PRINT    0
#define LOCAL_TIMER_PERIOD    100ms
#define CHAIN_NUM_MU          4
#define LOCAL_THREAD_NUM      2
#define LOCAL_WORKLOAD_MS     0
//--------------------------------
#define CH1_Timer1_Period_MS    50ms
#define CH3_Timer1_Period_MS    100ms
#define CH4_Timer1_Period_MS    200ms
#define CH5_Timer1_Period_MS    160ms
#define CH6_Timer1_Period_MS    1000ms
#define CH7_Timer1_Period_MS    200ms
#define CH8_Timer1_Period_MS    300ms
#define CH9_Timer1_Period_MS    400ms
#define CH10_Timer1_Period_MS   500ms

//--------------------------------
#define CH1_Timer1_Load_100US      30
#define CH1_Sub1_Load_100US        30
//--------------------------------
#define CH2_Sub1_Load_100US        49
#define CH2_Sub2_Load_100US        30
#define CH2_Sub3_Load_100US        41
//--------------------------------
#define CH3_Timer1_Load_100US      108
#define CH3_Sub1_Load_100US        126
#define CH3_Sub2_Load_100US        124
//--------------------------------
#define CH4_Timer1_Load_100US      221
#define CH4_Sub1_Load_100US        425
#define CH4_Sub2_Load_100US        424
#define CH4_Sub3_Load_100US        230


using namespace std::chrono_literals;
extern volatile int32_t cnter_val[50];
//---> Calibrate dummy task parameter

int main(int argc, char * argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  dummy_load_calibration();

#if (LOCAL_SCREEN_PRINT == 1)
#if (CHAIN_NUM_MU == 1)
  FILE *fp = freopen("results/CASE_STUDY/TWO_THREAD/ONE_CHAIN/LF_ONE.log", "w", stdout);
#endif
#if (CHAIN_NUM_MU == 2)
  FILE *fp = freopen("results/CASE_STUDY/TWO_THREAD/TWO_CHAINS/LF_TWO.log", "w", stdout);
#endif
#if (CHAIN_NUM_MU == 3)
  FILE *fp = freopen("results/CASE_STUDY/TWO_THREAD/THREE_CHAINS/LF_THREE.log", "w", stdout);
#endif
#if (CHAIN_NUM_MU == 4)
#if (LOCAL_THREAD_NUM == 1)
  FILE *fp = freopen("results/CASE_STUDY/FOUR_CHAINS/ONE_THREAD/LF_FOUR.log", "w", stdout);
#endif
#if (LOCAL_THREAD_NUM == 2)
  FILE *fp = freopen("results/CASE_STUDY/TWO_THREAD/FOUR_CHAINS/LF_FOUR.log", "w", stdout);
#endif
#if (LOCAL_THREAD_NUM == 3)
  FILE *fp = freopen("results/CASE_STUDY/Thread#3/Chain#4/LF_FOUR.log", "w", stdout);
#endif
#if (LOCAL_THREAD_NUM == 4)
  FILE *fp = freopen("results/CASE_STUDY/FOUR_CHAINS/FOUR_THREADS/LF_FOUR.log", "w", stdout);
#endif
#endif
  if(NULL == fp){
    printf("failed~\r\n");
    return -1;
  }
#endif

  rclcpp::init(argc, argv);
  rclcpp::executors::RTeXExecutor executor(rclcpp::ExecutorOptions(), LOCAL_THREAD_NUM);
#if (CHAIN_NUM_MU >= 1)
  // #Chain1.
  auto Chain1_Sensor1 = std::make_shared<Sensor>("Chain1_Sensor1", "LIDAR",                                1, 1, CH1_Timer1_Load_100US, CH1_Timer1_Period_MS);
  executor.add_node(Chain1_Sensor1);
  auto Chain1_Command1 = std::make_shared<Command>("Chain1_Command1", "LIDAR",                             1, 2, CH1_Sub1_Load_100US);
  executor.add_node(Chain1_Command1);
#endif
#if (CHAIN_NUM_MU >= 2)
  // #Chain2
  auto Chain2_Transfer1 = std::make_shared<Transfer>("Chain2_Transfer1", "LIDAR",  "POSE",                 2, 2, CH2_Sub1_Load_100US);
  executor.add_node(Chain2_Transfer1);
  auto Chain2_Transfer2 = std::make_shared<Transfer>("Chain2_Transfer2", "POSE", "LOCAL_COSTMAP",          2, 3, CH2_Sub2_Load_100US);
  executor.add_node(Chain2_Transfer2);
  auto Chain2_Command1 = std::make_shared<Command>("Chain2_Command1", "LOCAL_COSTMAP",                     2, 4, CH2_Sub3_Load_100US);
  executor.add_node(Chain2_Command1);
#endif
#if (CHAIN_NUM_MU >= 3)
  // #Chain3
  auto Chain3_Sensor1 = std::make_shared<Sensor>("Chain3_Sensor1", "CAMERA1",                              3, 1, CH3_Timer1_Load_100US, CH3_Timer1_Period_MS);
  executor.add_node(Chain3_Sensor1);
  auto Chain3_Transfer1 = std::make_shared<Transfer>("Chain3_Transfer1", "CAMERA1", "PRE_PROCESSING",      3, 2, CH3_Sub1_Load_100US);
  executor.add_node(Chain3_Transfer1);
  auto Chain3_Command1 = std::make_shared<Command>("Chain3_Command1", "PRE_PROCESSING",                    3, 3, CH3_Sub2_Load_100US);
  executor.add_node(Chain3_Command1);
#endif
#if (CHAIN_NUM_MU >= 4)
  // #Chain4
  auto Chain4_Sensor1 = std::make_shared<Sensor>("Chain4_Sensor1", "CAMERA2",                              4, 1, CH4_Timer1_Load_100US, CH4_Timer1_Period_MS);
  executor.add_node(Chain4_Sensor1);
  auto Chain4_Transfer1 = std::make_shared<Transfer>("Chain4_Transfer1", "CAMERA2", "DEPTH_ESTIMATION1",   4, 2, CH4_Sub1_Load_100US);
  executor.add_node(Chain4_Transfer1);
  auto Chain4_Transfer2 = std::make_shared<Transfer>("Chain4_Transfer2", "DEPTH_ESTIMATION1", "DEPTH_ESTIMATION2", 4, 3, CH4_Sub2_Load_100US);
  executor.add_node(Chain4_Transfer2);
  auto Chain4_Command1 = std::make_shared<Command>("Chain4_Command1", "DEPTH_ESTIMATION2",                  4, 4, CH4_Sub3_Load_100US);
  executor.add_node(Chain4_Command1);
#endif


  timerList_init();

  executor.spin();
  rclcpp::shutdown();

#if (LOCAL_SCREEN_PRINT == 1)
  fclose(fp);
#endif
  return 0;
}