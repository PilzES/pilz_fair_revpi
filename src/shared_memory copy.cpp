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

using namespace std::chrono_literals;

struct DataExchangeRead {
  bool automode;
  bool stop;
  bool warning;
};

struct DataExchangeWrite {
  float linear_x;
  float angular_z;
};

struct SyncDataExchange {
  bool sync;
};

class SharedMemory : public rclcpp::Node
{
  public:
    SharedMemory()
    : Node("shared_memory_node")
    {

      struct SyncDataExchange *pSync;
      struct DataExchangeRead *pRead;
      struct DataExchangeWrite *pWrite;
      char sSharedMemSyncName[100] = "_Sync_shm";
      char sSharedMemReadName[100] = "_CODESYS_SharedMemoryTest_Write";
      char sSharedMemWriteName[100] = "_CODESYS_SharedMemoryTest_Read";
      int fdSync = shm_open(sSharedMemSyncName, O_RDWR, S_IRWXU | S_IRWXG);
      pSync = static_cast<struct SyncDataExchange *>(mmap(0, sizeof(struct SyncDataExchange), PROT_READ | PROT_WRITE, MAP_SHARED, fdSync, 0));
      pSync->sync = 1;
      int fdRead = shm_open(sSharedMemReadName, O_RDWR, S_IRWXU | S_IRWXG);
      int fdWrite = shm_open(sSharedMemWriteName, O_RDWR, S_IRWXU | S_IRWXG);
      pRead = static_cast<struct DataExchangeRead *>(mmap(0, sizeof(struct DataExchangeRead), PROT_READ, MAP_SHARED, fdRead, 0));
      pWrite = static_cast<struct DataExchangeWrite *>(mmap(0, sizeof(struct DataExchangeWrite), PROT_READ | PROT_WRITE, MAP_SHARED, fdWrite, 0));

      pWrite->linear_x = 50;
      pWrite->angular_z = 60;

      bool manual = pRead->automode;

      munmap(pRead, sizeof(struct DataExchangeRead));
      close(fdRead);
      munmap(pWrite, sizeof(struct DataExchangeWrite));
      close(fdWrite);
      pSync->sync = 0;
      munmap(pSync, sizeof(struct SyncDataExchange));
      close(fdSync);

      std::cout << "The number is: " << manual << std::endl;
      exit(0);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SharedMemory>());
  rclcpp::shutdown();
  return 0;
}

