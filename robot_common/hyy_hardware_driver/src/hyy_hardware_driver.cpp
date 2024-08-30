#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <csignal>
#include <atomic>
#include <sys/time.h>
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/thread_priority.hpp"
#include "device_interface/DeviceDriver/device_timer.h"
#include "device_interface/Base/RobotSystem.h"

using namespace std::chrono_literals;

namespace{
  int const kSchedPriority = 50;
  std::atomic<bool> is_shutting_down(false);
}

void signal_handler(int signal){
  if (signal == SIGINT){
    is_shutting_down.store(true);
  }
}

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  std::signal(SIGINT, signal_handler);

  uint32_t communication_time = 0;
  int Priority = 38;

  std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";
  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name);

  if (cm->get_parameter("communication_time", communication_time)){
    RCLCPP_INFO(cm->get_logger(), "Parameter \"communication_time\" is %d ns", communication_time);
  }else{
    RCLCPP_ERROR(cm->get_logger(), "Parameter \"communication_time\" not find");
  }
  if (0 != HYYRobotBase::initPriority(Priority)){
    RCLCPP_ERROR(cm->get_logger(), "program priority set failure!");
    return 0;
  }else{
    RCLCPP_INFO(cm->get_logger(), "program priority set to %d.", Priority);
  }

  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());

  std::thread cm_thread(
    [cm, communication_time]()
    {
      if (realtime_tools::has_realtime_kernel()){
        if (!realtime_tools::configure_sched_fifo(kSchedPriority)){
          RCLCPP_WARN(cm->get_logger(), "Could not enable FIFO RT scheduling policy");
        }
      }else{
        RCLCPP_INFO(cm->get_logger(), "RT kernel is recommended for better performance");
      }

      // initial timer
      HYYRobotBase::RTimer timer;
      int multiple = 1;
      if (0 != HYYRobotBase::initUserTimer(&timer, 0, multiple)){
        RCLCPP_ERROR(cm->get_logger(), "device_timer start failure!");
      }else{
        RCLCPP_INFO(cm->get_logger(), "device_timer start with %dx bus cycles.", multiple);
      }

      // set loop time.
      rclcpp::Duration d_time(0, communication_time);
      // double time2 = d_time.nanoseconds() / 1e9;
      // loop initial time.
      rclcpp::Time curtime = cm->now();

      //
      struct timeval start, end;
      // double __dtime = 0;
      // double time_ = 0;
      // double time_initial = curtime.nanoseconds() / 1e9;

      while (rclcpp::ok() && !is_shutting_down.load())
      {
        // get loop start time.

        // execute update loop.
        cm->read(curtime, d_time);
        cm->update(curtime, d_time);
        cm->write(curtime, d_time);
        curtime += d_time;

        // Loop timing.
        HYYRobotBase::userTimer(&timer);
        gettimeofday(&end, NULL);

        
        // time_ = (curtime.nanoseconds() / 1e9) - time_initial;
        // __dtime = (end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec);
        // RCLCPP_INFO(cm->get_logger(), "time: %.4fs, costtime :%.4fms, time2 :%.4fs, communication_time: %d", time_, __dtime, time2, communication_time);
        // get loop end time.
        gettimeofday(&start, NULL);

        // calculate loop timecost
        // if (__dtime > (communication_time / 1000 * 0.8)){
        //   RCLCPP_INFO(cm->get_logger(), "Over time: loop time is %d us", 1000-__dtime);
        // }
      }
    }
  );

  executor->add_node(cm);
  while(!is_shutting_down.load()){
    executor->spin_some();
    std::this_thread::sleep_for(1ms);
  }

  RCLCPP_INFO(cm->get_logger(), "Perform shutdown after 3 seconds.");
  std::this_thread::sleep_for(3s);

  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}