#include <overworld/SituationAssessor.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");

  owds::SituationAssessor robot_situation_assessor("pr2_robot", true);

  robot_situation_assessor.run();
}