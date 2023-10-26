#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

struct DataExchangeRead {
  bool automode;
  bool stop;
  bool warning;
};

struct DataExchangeWrite {
  double linear_x;
  double angular_z;
};

struct SyncDataExchange {
  bool sync;
};

class SharedMemory : public rclcpp::Node
{
public:
  SharedMemory() : Node("shared_memory_node")
  {
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        processCmdVelMessage(msg);
      });
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

  void processCmdVelMessage(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    try{

      double lin_x = msg->linear.x;
      double ang_z = msg->angular.z;

      RCLCPP_INFO(get_logger(), "Received cmd_vel message: Linear X = %.2f, Angular Z = %.2f", lin_x, ang_z);

      struct SyncDataExchange *pSync;
      struct DataExchangeRead *pRead;
      struct DataExchangeWrite *pWrite;
      char sSharedMemSyncName[100] = "_Sync_shm";
      char sSharedMemReadName[100] = "_CODESYS_SharedMemoryTest_Write";
      char sSharedMemWriteName[100] = "_CODESYS_SharedMemoryTest_Read";
      int fdSync = shm_open(sSharedMemSyncName, O_RDWR, S_IRWXU | S_IRWXG);
      int fdRead = shm_open(sSharedMemReadName, O_RDWR, S_IRWXU | S_IRWXG);
      int fdWrite = shm_open(sSharedMemWriteName, O_RDWR, S_IRWXU | S_IRWXG);
      pRead = static_cast<struct DataExchangeRead *>(mmap(0, sizeof(struct DataExchangeRead), PROT_READ, MAP_SHARED, fdRead, 0));
      pWrite = static_cast<struct DataExchangeWrite *>(mmap(0, sizeof(struct DataExchangeWrite), PROT_READ | PROT_WRITE, MAP_SHARED, fdWrite, 0));
      pSync = static_cast<struct SyncDataExchange *>(mmap(0, sizeof(struct SyncDataExchange), PROT_READ | PROT_WRITE, MAP_SHARED, fdSync, 0));
      
      if (fdSync == -1 || fdRead == -1 || fdWrite == -1) {
            perror("shm_open");
            return; 
        }

      pSync->sync = 1;

      pWrite->linear_x = 50.0;
      pWrite->angular_z = 60.0;

      munmap(pRead, sizeof(struct DataExchangeRead));
      munmap(pWrite, sizeof(struct DataExchangeWrite));
      munmap(pSync, sizeof(struct SyncDataExchange));
      pSync->sync = 0;
      close(fdRead);
      close(fdWrite);
      close(fdSync);
    }
    catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception in processCmdVelMessage: %s", e.what());
    }
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SharedMemory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}