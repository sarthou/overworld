#include <overworld/SituationAssessor.h>

#include "overworld/Utility/ShellDisplay.h"

#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

void handler(int sig)
{
  void *array[10];
  size_t size;

  size = backtrace(array, 10);

  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

int main(int argc, char** argv)
{
  signal(SIGSEGV, handler);
  signal(SIGABRT, handler);
  ros::init(argc, argv, "overworld");

  if(argc < 3)
  {
    owds::ShellDisplay::error("A configuration file and the robot name should be provided");
    return -1;
  }

  std::string config_path = std::string(argv[1]);
  std::string robot_name = std::string(argv[2]);

  owds::SituationAssessor robot_situation_assessor(robot_name, config_path, true);

  robot_situation_assessor.run();

  return 0;
}