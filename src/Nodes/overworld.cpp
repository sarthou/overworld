#include <overworld/SituationAssessor.h>

#include "overworld/Utility/ShellDisplay.h"
#include "overworld/Utility/Parameters.h"

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

  owds::Parameters params;
  params.insert(owds::Parameter("config_path", {"-c", "--config"}));
  params.insert(owds::Parameter("robot_name", {"-n", "--name"}));
  params.insert(owds::Parameter("simulate", {"-s", "--simulate"}, {"true"}));

  bool valid_parameters = params.set(argc, argv);
  params.display();
  if(valid_parameters == false)
  {
    owds::ShellDisplay::error("Some parameters have not been setted. Overworld will shutdown.");
    return -1;
  }

  owds::SituationAssessor robot_situation_assessor(params.at("robot_name").getFirst(),
                                                   params.at("config_path").getFirst(),
                                                   params.at("simulate").getFirst() == "true",
                                                   true);

  robot_situation_assessor.run();

  return 0;
}