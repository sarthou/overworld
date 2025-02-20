#include <execinfo.h>
#include <overworld/SituationAssessor.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>

#include "overworld/Utils/Parameters.h"
#include "overworld/Utils/ShellDisplay.h"
#include "overworld/Engine/Engine.h"

void handler(int sig)
{
  void* array[10];
  size_t size;

  size = backtrace(array, 10);

  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

std::unordered_map<std::string, owds::Window*> windows;
std::unordered_map<std::string, owds::SituationAssessor*> human_assessors;
owds::SituationAssessor* robot_assessor;

std::unordered_set<std::string> requests; // TODO should be protected

void requestAssessorCreation(const std::string& human_name)
{
  requests.insert(human_name);
}

void robotAssessorThread(owds::Window* window, owds::SituationAssessor* assessor)
{
  assessor->initWorld(window);
  assessor->initAssessor();
  assessor->setCreationCallback(requestAssessorCreation);

  assessor->run();
}

int main(int argc, char** argv)
{
  signal(SIGSEGV, handler);
  signal(SIGABRT, handler);
  ros::init(argc, argv, "overworld");

  owds::Renderer::init();

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

  std::string robot_name = params.at("robot_name").getFirst();

  robot_assessor = new owds::SituationAssessor (robot_name,
                                                params.at("config_path").getFirst(),
                                                std::stod(params.at("assessment frequency").getFirst()),
                                                std::stod(params.at("simulation frequency").getFirst()),
                                                params.at("simulate").getFirst() == "true",
                                                true);

  windows.emplace(robot_name, new owds::Window(robot_name));
  std::thread tread(&robotAssessorThread, windows.at(robot_name), robot_assessor);

  while(ros::ok())
  {
    owds::Window::pollEvent();
    usleep(1000);

    if(requests.empty() == false)
    {
      std::unordered_set<std::string> rqts = requests;
      requests.clear();

      for(const auto& human_name : rqts)
      {
        if(windows.find(human_name) != windows.end())
          continue;

        windows.emplace(human_name, new owds::Window(human_name));
        robot_assessor->createHumanAssessor(human_name, windows.at(human_name));
      }
    }
  }

  tread.join();

  owds::Renderer::release();

  return 0;
}