#ifndef RCLCPP__EXECUTORS__LOCK_FREE_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__LOCK_FREE_EXECUTOR_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/detail/mutex_two_priorities.hpp"
#include "rclcpp/executors/exp_config.hpp"

namespace rclcpp
{
namespace executors
{

class RTeXExecutor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RTeXExecutor)

  RCLCPP_PUBLIC
  explicit RTeXExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    size_t number_of_threads = 0,
    bool yield_before_execute = false,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  virtual ~RTeXExecutor();

  RCLCPP_PUBLIC
  void
  spin() override;

  RCLCPP_PUBLIC
  size_t
  get_number_of_threads();

protected:
  RCLCPP_PUBLIC
  void
  run(size_t this_thread_number);

private:
  RCLCPP_DISABLE_COPY(RTeXExecutor)


  std::mutex wait_mutex_;
  size_t number_of_threads_;
  bool yield_before_execute_;
  std::chrono::nanoseconds next_exec_timeout_;

};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__LOCK_FREE_EXECUTOR_HPP_