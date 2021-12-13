#include <overworld/SituationAssessor.h>

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

  if(argc < 2)
  {
    std::cout << "A configuration file should be provided" << std::endl;
    return -1;
  }

  std::string config_path = std::string(argv[1]);

  owds::SituationAssessor robot_situation_assessor("pr2_robot", config_path, true);

  robot_situation_assessor.run();

  return 0;
}