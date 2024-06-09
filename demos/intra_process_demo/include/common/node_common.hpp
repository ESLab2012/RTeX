#ifndef DEMOS__NODE_COMMON_H
#define DEMOS__NODE_COMMON_H

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

/// counter
volatile int32_t cnter_val[50] = {0,0,0,0,0,0};  

/// MARK: `Sensor` Node structure definition.
struct Sensor : public rclcpp::Node {
  Sensor(const std::string & node_name, const std::string & output_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, std::chrono::duration<int,std::milli> period_ms)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output_topic, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;

    auto callback = [captured_pub, node_idx, chain_idx, exe_time_100us]() -> void {
        volatile uint64_t ts_start = get_clocktime();     // NOTE: --> start
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }
        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
        std::thread::id thread_id = std::this_thread::get_id();
        msg->data = cnter_val[chain_idx-1]++;
        dummy_load_100us(exe_time_100us);
        volatile uint64_t ts_end = get_clocktime();       // NOTE: <-- end

        printf("\n|TID:%u|-->[Chain%u-Sensor%u.exe:%u(100us)]-->[Start@%lu-End@%lu-Cost:%lu(ns) and Published msg: %d]. \r\n", 
                (*(uint32_t*)&thread_id),
                chain_idx,
                node_idx,
                exe_time_100us,
                ts_start,
                ts_end,
                ts_end-ts_start,
                msg->data
              );
        pub_ptr->publish(std::move(msg));
      };
    timer_ = this->create_wall_timer(period_ms, callback);


    readyList.first_insert(chain_idx, TYPE_TIMER, (uintptr_t)timer_.get());
    rcl_timer_t rcltimer=rcl_get_zero_initialized_timer();
    auto timer_handle=timer_->get_timer_handle();
    rcltimer.impl=timer_handle->impl;
    int64_t next_call_time;
    if(rcl_timer_get_next_call_time(&rcltimer,&next_call_time)!=RCL_RET_OK)
      printf("get next_call_time false!\n");
    map_global_timerList[map_num++] = {next_call_time , (uintptr_t)timer_.get()};
    Timer_num++;
  }
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/// MARK: `Transfer` Node structure definition.
struct Transfer : public rclcpp::Node {
  Transfer(const std::string & node_name, const std::string & input_topic, const std::string & output_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output_topic, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;

    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input_topic,
      10,
      [captured_pub, node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
          volatile uint64_t ts_start = get_clocktime();     // NOTE: --> start
          auto pub_ptr = captured_pub.lock();
          if (!pub_ptr) {
            return;
          }
          std_msgs::msg::Int32::UniquePtr msg_agent(new std_msgs::msg::Int32());
          msg_agent->data = msg->data;
          std::thread::id thread_id = std::this_thread::get_id();
          dummy_load_100us(exe_time_100us);
          volatile uint64_t ts_end = get_clocktime();       // NOTE: <-- end

          printf("\n|TID:%u|-->[Chain%u-Transfer%u.exe:%u(100us)]-->[Start@%lu-End@%lu-Cost:%lu(ns) and Route msg: %d]. \r\n", 
                  (*(uint32_t*)&thread_id),
                  chain_idx,
                  node_idx,
                  exe_time_100us,
                  ts_start,
                  ts_end,
                  ts_end-ts_start,
                  msg_agent->data
                );
          pub_ptr->publish(std::move(msg_agent));
      }
    );


    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub_.get());
    auto sub_handle=sub_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub_handle.get()]);
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

/// MARK: `Command` Node structure definition.
struct Command : public rclcpp::Node {
  Command(const std::string & node_name, const std::string & input_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input_topic,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
      // NOTE: --> start
          volatile uint64_t ts_start = get_clocktime();
          std::thread::id thread_id = std::this_thread::get_id();
          dummy_load_100us(exe_time_100us);
          volatile uint64_t ts_end = get_clocktime();
      // NOTE: <-- end
          printf("\n|TID:%u|-->[Chain%u-Command%u.exe:%u(100us)]-->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d]. \r\n", 
                  (*(uint32_t*)&thread_id),
                  chain_idx,
                  node_idx,
                  exe_time_100us,
                  ts_start,
                  ts_end,
                  ts_end-ts_start,
                  msg->data
                );
      }
    );

    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub_.get());
    auto sub_handle=sub_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub_handle.get()]);
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};




/// MARK: `Fusion3` Node structure definition.
volatile int32_t fusion3_cache_[3] = {-99, -99, -99};
volatile int32_t fusion3_round = 0;
struct Fusion3 : public rclcpp::Node {
  Fusion3( const std::string & node_name, 
          const std::string & in_topic1, const std::string & in_topic2, const std::string & in_topic3,
          uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us
        )
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {

    sub1_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic1,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion3_cache_[0] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<3; temp_idx++) {
          if (fusion3_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion3_sub1.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion3_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion3_sub1.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion3]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion3_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<3; temp_idx++) {
            fusion3_cache_[temp_idx] = -99;
          }
          fusion3_round++;
        }
      }
    );

    sub2_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic2,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion3_cache_[1] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<3; temp_idx++) {
          if (fusion3_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion3_sub2.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion3_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion3_sub2.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion3]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion3_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<3; temp_idx++) {
            fusion3_cache_[temp_idx] = -99;
          }
          fusion3_round++;
        }
      }
    );

    sub3_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic3,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion3_cache_[2] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<3; temp_idx++) {
          if (fusion3_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion3_sub3.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion3_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion3_sub3.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion3]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion3_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<3; temp_idx++) {
            fusion3_cache_[temp_idx] = -99;
          }
          fusion3_round++;
        }
      }
    );

    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub1_.get());
    auto sub1_handle=sub1_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub1_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub2_.get());
    auto sub2_handle=sub2_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub2_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub3_.get());
    auto sub3_handle=sub3_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub3_handle.get()]);
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub2_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub3_;
};





/// MARK: `Fusion5`
volatile int32_t fusion5_cache_[5] = {-99, -99, -99, -99, -99};
volatile int32_t fusion5_round = 0;
struct Fusion5 : public rclcpp::Node {
  Fusion5( const std::string & node_name, 
          const std::string & in_topic1, const std::string & in_topic2, const std::string & in_topic3, const std::string & in_topic4, const std::string & in_topic5,
          uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us
        )
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    sub1_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic1,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion5_cache_[0] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<5; temp_idx++) {
          if (fusion5_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion5_sub1.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion5_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion5_sub1.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion5]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion5_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<5; temp_idx++) {
            fusion5_cache_[temp_idx] = -99;
          }
          fusion5_round++;
        }
      }
    );
    sub2_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic2,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion5_cache_[1] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<5; temp_idx++) {
          if (fusion5_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion5_sub2.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion5_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion5_sub2.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion5]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion5_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<5; temp_idx++) {
            fusion5_cache_[temp_idx] = -99;
          }
          fusion5_round++;
        }
      }
    );
    sub3_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic3,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion5_cache_[2] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<5; temp_idx++) {
          if (fusion5_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion5_sub3.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion5_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion5_sub3.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion5]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion5_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<5; temp_idx++) {
            fusion5_cache_[temp_idx] = -99;
          }
          fusion5_round++;
        }
      }
    );
    sub4_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic4,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion5_cache_[3] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<5; temp_idx++) {
          if (fusion5_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion5_sub4.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion5_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion5_sub4.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion5]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion5_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<5; temp_idx++) {
            fusion5_cache_[temp_idx] = -99;
          }
          fusion5_round++;
        }
      }
    );
    sub5_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic5,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion5_cache_[4] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<5; temp_idx++) {
          if (fusion5_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion5_sub5.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion5_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion5_sub5.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion5]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion5_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<5; temp_idx++) {
            fusion5_cache_[temp_idx] = -99;
          }
          fusion5_round++;
        }
      }
    );
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub1_.get());
    auto sub1_handle=sub1_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub1_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub2_.get());
    auto sub2_handle=sub2_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub2_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub3_.get());
    auto sub3_handle=sub3_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub3_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub4_.get());
    auto sub4_handle=sub4_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub4_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub5_.get());
    auto sub5_handle=sub5_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub5_handle.get()]);
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub2_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub3_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub4_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub5_;
};


/// MARK: `Fusion10`
volatile int32_t fusion10_cache_[10] = {-99, -99, -99, -99, -99, -99, -99, -99, -99, -99};
volatile int32_t fusion10_round = 0;
struct Fusion10 : public rclcpp::Node {
  Fusion10( const std::string & node_name, 
          const std::string & in_topic1, const std::string & in_topic2, const std::string & in_topic3, const std::string & in_topic4, const std::string & in_topic5, const std::string & in_topic6, const std::string & in_topic7, const std::string & in_topic8, const std::string & in_topic9, const std::string & in_topic10,
          uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us
        )
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    sub1_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic1,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion10_cache_[0] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<10; temp_idx++) {
          if (fusion10_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub1.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub1.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion10]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<10; temp_idx++) {
            fusion10_cache_[temp_idx] = -99;
          }
          fusion10_round++;
        }
      }
    );
    sub2_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic2,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion10_cache_[1] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<10; temp_idx++) {
          if (fusion10_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub2.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub2.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion10]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<10; temp_idx++) {
            fusion10_cache_[temp_idx] = -99;
          }
          fusion10_round++;
        }
      }
    );
    sub3_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic3,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion10_cache_[2] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<10; temp_idx++) {
          if (fusion10_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub3.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub3.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion10]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<10; temp_idx++) {
            fusion10_cache_[temp_idx] = -99;
          }
          fusion10_round++;
        }
      }
    );
    sub4_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic4,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion10_cache_[3] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<10; temp_idx++) {
          if (fusion10_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub4.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub4.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion10]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<10; temp_idx++) {
            fusion10_cache_[temp_idx] = -99;
          }
          fusion10_round++;
        }
      }
    );
    sub5_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic5,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion10_cache_[4] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<10; temp_idx++) {
          if (fusion10_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub5.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub5.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion10]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<10; temp_idx++) {
            fusion10_cache_[temp_idx] = -99;
          }
          fusion10_round++;
        }
      }
    );
    sub6_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic6,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion10_cache_[5] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<10; temp_idx++) {
          if (fusion10_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub6.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub6.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion10]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<10; temp_idx++) {
            fusion10_cache_[temp_idx] = -99;
          }
          fusion10_round++;
        }
      }
    );
    sub7_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic7,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion10_cache_[6] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<10; temp_idx++) {
          if (fusion10_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub7.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub7.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion10]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<10; temp_idx++) {
            fusion10_cache_[temp_idx] = -99;
          }
          fusion10_round++;
        }
      }
    );
    sub8_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic8,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion10_cache_[7] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<10; temp_idx++) {
          if (fusion10_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub8.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub8.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion10]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<10; temp_idx++) {
            fusion10_cache_[temp_idx] = -99;
          }
          fusion10_round++;
        }
      }
    );
    sub9_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic9,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion10_cache_[8] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<10; temp_idx++) {
          if (fusion10_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub9.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub9.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion10]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<10; temp_idx++) {
            fusion10_cache_[temp_idx] = -99;
          }
          fusion10_round++;
        }
      }
    );
    sub10_ = this->create_subscription<std_msgs::msg::Int32>(
      in_topic10,
      10,
      [node_idx, chain_idx, exe_time_100us](std_msgs::msg::Int32::UniquePtr msg) {
        volatile uint64_t ts_start,ts_end;
        ts_start = get_clocktime();   // NOTE: --> start
        std::thread::id thread_id = std::this_thread::get_id();
        fusion10_cache_[9] = msg->data;
        int temp_idx = 0;
        bool success = true;
        for (temp_idx=0; temp_idx<10; temp_idx++) {
          if (fusion10_cache_[temp_idx] < 0) {
            success = false;
          }
        }
        if (success == false) {
          ts_end = get_clocktime();   // NOTE: <-- end1
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub10.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Wait other msgs]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
        } else {
          dummy_load_100us(exe_time_100us);
          ts_end = get_clocktime();   // NOTE: <-- end2
          printf("\n|TID:%u|-->[Chain%u-Fusion10_sub10.exe:%u(100us)]->[Round%d]->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d and Start Fusion10]. \r\n", 
                (*(uint32_t*)&thread_id), chain_idx, exe_time_100us, fusion10_round, ts_start, ts_end, (ts_end-ts_start), msg->data);
          for (temp_idx=0; temp_idx<10; temp_idx++) {
            fusion10_cache_[temp_idx] = -99;
          }
          fusion10_round++;
        }
      }
    );
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub1_.get());
    auto sub1_handle=sub1_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub1_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub2_.get());
    auto sub2_handle=sub2_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub2_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub3_.get());
    auto sub3_handle=sub3_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub3_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub4_.get());
    auto sub4_handle=sub4_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub4_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub5_.get());
    auto sub5_handle=sub5_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub5_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub6_.get());
    auto sub6_handle=sub6_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub6_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub7_.get());
    auto sub7_handle=sub7_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub7_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub8_.get());
    auto sub8_handle=sub8_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub8_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub9_.get());
    auto sub9_handle=sub9_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub9_handle.get()]);
    readyList.first_insert(chain_idx, TYPE_SUB, (uintptr_t)sub10_.get());
    auto sub10_handle=sub10_->get_subscription_handle();
    readyList.first_insert(chain_idx, TYPE_WAITABLE, sub_correspond_waitable[(uintptr_t)sub10_handle.get()]);
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub2_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub3_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub4_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub5_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub6_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub7_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub8_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub9_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub10_;
};

#endif