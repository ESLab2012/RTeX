## Overview

This repository presents the implementation of our [RTeX for ROS 2](https://ieeexplore.ieee.org/document/10477454). RTeX is designed based on a lock-free data structure (*readyList*), to efficiently manage the ready callbacks, supporting concurrent insertion and removal of ready callbacks without any lock contention. Furthermore, RTeX also supports chain-aware priority scheduling, and timing predictable run-time behavior.
RTeX is fully implemented in the most recent ROS 2 version [Humble](https://docs.ros.org/en/humble/index.html).

## Directory/File Structure

We added a new executor class `RTeX_executor` in folder *rclccp/executors*. 
Below is an explanation of the directories/files found in this repo. Note that only the modified/added files are presented.


```tex
src/
├─ demos/     /* Experiments and Case study used */
│  ├─ intra_process_demo/
│  │  ├─ include/
│  │  │  ├─ common/
│  │  │  │  ├─ dummy_load.hpp
│  │  │  │  ├─ node_common.hpp
│  │  ├─ src/
│  │  │  ├─ case_study/
│  │  │  │  ├─ case_study.cpp
│  │  │  ├─ common/
│  │  │  │  ├─ dummy_load.cpp
│  │  │  ├─ two_node_pipeline/
│  │  │  │  ├─ 1_to_nsubs_new.cpp
│  │  │  │  ├─ n_to_1sub_new.cpp
├─ rclcpp/
│  ├─ include/
│  │  ├─ executors/
│  │  │  ├─ exp_config.hpp                /* NEW: Macros used for experiments */
│  │  │  ├─ RTeX_executor.hpp             /* NEW: Class definition for RTeX executor*/
│  │  ├─ experimental/
│  │  │  ├─ intra_process_manager.hpp     /* MODIFIED: hook ROS 2 intra-process API to insert callback */
│  │  ├─ lockFree_readyList/
│  │  │  ├─ lockFree_readyList.hpp        /* NEW: Data structure for readyList */
│  │  │  ├─ lockFree_timerMonitor.hpp     /* NEW: Data structure for timerMonitor */
|	 |  ├─ timer.hpp
│  ├─ src/
│  │  ├─ executors/
│  │  │  ├─ lockFree_readyList.cpp        /* NEW: internal API of readyList */
│  │  │  ├─ lockFree_timerMonitor.cpp     /* NEW: internal API of timerMonitor */
│  │  │  ├─ RTeX_executor.cpp             /* NEW: API of RTeX executor */
│  │  ├─ executor.cpp                     /* MODIFIED: add some internal API to base Class:`Executor` */
├─ rcl/
│  ├─ include/
|	 |  ├─ timer.h  
│  ├─ src/
|	 |  ├─ timer.c  
```


## Getting Started

### Description

This repository comprises three main components: the `rcl` package, the `rclcpp` package for implementing RTeX, and `demo` code for testing the real-time performance of RTeX.

### Prerequisites

RTeX is based on the latest version of ROS 2 Humble. Before compiling and building RTeX, you need to install ROS 2 on your Ubuntu system. Follow the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) for detailed instructions.

### Build

1. Create a ROS 2 workspace. Detailed instructions can be found [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

2. Download or clone this repository into the workspace.

3. Build the workspace with the command:

   ```bash
   colcon build --allow-overriding rcl rclcpp
   ```

4. Source the local packages:

   ```bash
   source ./install/local_setup.zsh
   ```

### Running

1. RTeX binds several executor threads to specific CPU cores with fixed frequencies for execution. Therefore, before running this project, set a fixed CPU frequency and isolate the CPU cores. For details, you can refer to the [reference-system guide](https://github.com/ros-realtime/reference-system). In current setting, core 0 ~ 5 are used for RTeX threads.

2. If everything is set up correctly, you can run the given example:

   ```bash
   ros2 run intra_process_demo 1_to_nsubs_new
   ```

### Usage

1. Create executor and callbacks, e.g., timer_callback, regular_callback:

   ```c++
   rclcpp::executors::RTeXExecutor executor(rclcpp::ExecutorOptions(), LOCAL_THREAD_NUM);
   ```

   Here, `LOCAL_THREAD_NUM` is the thread number in a RTeX executor. Intra-process communication is configured in `rclcpp::ExecutorOptions()`.

   ```c++
   auto timer_callback = std::make_shared<StartNode>("callback_name", "pub_topic_name", chain_idx, callback_idx, exe_time(msec), period(msec));
   executor.add_node(timer_callback);
   
   auto regular_callback = std::make_shared<IntermediatedNode>("callback_name", "sub_topic_name", "pub_topic_name", chain_idx, callback_idx, exe_time(msec));
   executor.add_node(regular_callback);
   ```

   - `chain_idx`: Index of the chain, smaller index chains have higher priority.
   - `callback_idx`: Index of the callback within the chain
   - `sub_topic_name`: The name of subscription topic
   - `pub_topic_name`: The name of publishing topic
   - `exe_time`: Execution time of the callback in milliseconds
   - `period`: Period of timer callback in milliseconds

2. Initialize the timerMonitor by

   ```c++
   timerList_init();
   ```

3. Finally, spin the executor

   ```c++
   executor.spin();
   ```
