#include <execinfo.h>
#include <overworld/SituationAssessor.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "overworld/Utility/Parameters.h"
#include "overworld/Utility/ShellDisplay.h"

void handler(int sig)
{
  void* array[10];
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
  params.insert(owds::Parameter("assessment frequency", {"-af", "--assessment-frequency"}, {"17"}));
  params.insert(owds::Parameter("simulation frequency", {"-sf", "--simulation-frequency"}, {"70"}));

  bool valid_parameters = params.set(argc, argv);
  params.display();
  if(valid_parameters == false)
  {
    owds::ShellDisplay::error("Some parameters have not been setted. Overworld will shutdown.");
    return -1;
  }

  owds::SituationAssessor robot_situation_assessor(params.at("robot_name").getFirst(),
                                                   params.at("config_path").getFirst(),
                                                   std::stod(params.at("assessment frequency").getFirst()),
                                                   std::stod(params.at("simulation frequency").getFirst()),
                                                   params.at("simulate").getFirst() == "true",
                                                   true);

  robot_situation_assessor.run();

  return 0;
}