#include "rclcpp/executors/RTeX_executor.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <unistd.h>
#include "rcpputils/scope_exit.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/executors/exp_config.hpp"
#include "rclcpp/lockFree_readyList/lockFree_readyList.hpp"
#include "rclcpp/lockFree_readyList/lockFree_timerMonitor.hpp"

using rclcpp::detail::MutexTwoPriorities;
using rclcpp::executors::RTeXExecutor;

RTeXExecutor::RTeXExecutor(
    const rclcpp::ExecutorOptions & options,
    size_t number_of_threads,
    bool yield_before_execute,
    std::chrono::nanoseconds next_exec_timeout)
: rclcpp::Executor(options),
    yield_before_execute_(yield_before_execute),
    next_exec_timeout_(next_exec_timeout)
{
    number_of_threads_ = number_of_threads ? number_of_threads : std::thread::hardware_concurrency();
    if (number_of_threads_ == 0) {
        number_of_threads_ = 1;
    }
    bool success = false;
    success = lockFree_init_MDList();
    if (!success) {
        printf("\n[ERROR] Executor creation failed!");
    }
}

RTeXExecutor::~RTeXExecutor() {}

void
RTeXExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  {
    std::lock_guard wait_lock{wait_mutex_};
    for (; thread_id < number_of_threads_ - 1; ++thread_id) {
      auto func = std::bind(&RTeXExecutor::run, this, thread_id);
      threads.emplace_back(func);
    }
  }
  run(thread_id);
  for (auto & thread : threads) {
    thread.join();
  }
}

std::mutex mtx;
size_t
RTeXExecutor::get_number_of_threads()
{
  return number_of_threads_;
}

void
RTeXExecutor::run(size_t this_thread_number)
{
  // CPU binding...
  cpu_set_t mask;
  CPU_ZERO(&mask);
  if (this_thread_number<=5) {
    CPU_SET(this_thread_number+5, &mask);
  }

  if(sched_setaffinity(0, sizeof(mask), &mask) < 0){
    printf("[ERROR] in sched_setaffinity() \r\n");
    while(1);
  }
  
  printf("|RTeX-->>>|Executor thread created on core %ld !\r\n", this_thread_number);
  std::thread::id thread_id = std::this_thread::get_id();

  // for safe creation
  std::unique_lock<std::mutex> lock(mtx);

  // assign a local `timerMonitor` for each thread
  if(TID_to_MAP.insert({(*(uint32_t*)&thread_id),TID_MAP[this_thread_number]}).second == false) {
    printf("[ERROR] in timerMonitor setup for thread %u !\r\n",(*(uint32_t*)&thread_id));
  }
  // assign a local `cStack` for each thread
  if(readyList.TID_to_cStack.insert({(*(uint32_t*)&thread_id),readyList.global_cStack[this_thread_number]}).second == false) {
    printf("[ERROR] in cStack setup for thread %u !\r\n",(*(uint32_t*)&thread_id));
  }

  lock.unlock();

  // RTeX main loop
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_exec;
    if (!RTeX_get_next_executable(any_exec, next_exec_timeout_)) {
      continue;
    }
    if (yield_before_execute_) {
      std::this_thread::yield();
    }
    RTeX_execute_any_executable(any_exec);
    // Clear the callback_group to prevent the AnyExecutable destructor from, resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
  }
}