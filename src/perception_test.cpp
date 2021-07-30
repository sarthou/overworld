#include "overworld/Perception/PerceptionModuleBase.h"
#include "overworld/Perception/EntitiesPerceptionManager.h"

#include "overworld/StampedStringTest.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>

class PerceptionTest : public owds::PerceptionModuleBase<owds::Entity, std::vector<int>>
{
public:
  PerceptionTest() = default;
  virtual ~PerceptionTest() = default;

private:
  void perceptionCallback(const std::vector<int>& msg)
  {
    std::cout << "PerceptionTest: ";
    for(auto i : msg)
      std::cout << i << " ";
    std::cout << std::endl;
  }
};

class RosPerceptionTest : public owds::PerceptionModuleRosBase<owds::Entity, std_msgs::String>
{
public:
  RosPerceptionTest(ros::NodeHandle* n) : PerceptionModuleRosBase(n, "owds/pecpetion_test_str") {}
  virtual ~RosPerceptionTest() = default;

private:
  void perceptionCallback(const std_msgs::String& msg)
  {
    std::cout << "RosPerceptionTest: " << msg.data << std::endl;
  }
};

class RosPerceptionSyncTest : public owds::PerceptionModuleRosSyncBase<owds::Entity, overworld::StampedStringTest, overworld::StampedStringTest>
{
public:
  RosPerceptionSyncTest(ros::NodeHandle* n) : PerceptionModuleRosSyncBase(n, "owds/sync/pecpetion_test_str_0", "owds/sync/pecpetion_test_str_1") {}
  virtual ~RosPerceptionSyncTest() = default;

private:
  void perceptionCallback(const overworld::StampedStringTest& msg_1, const overworld::StampedStringTest& msg_2)
  {
    std::cout << "RosPerceptionSyncTest: " << msg_1.data << " : " << msg_2.data << std::endl;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  PerceptionTest test_perception;
  RosPerceptionTest test_ros_perception(&n);
  RosPerceptionSyncTest test_ros_perception_sync(&n);

  owds::EntitiesPerceptionManager<owds::Entity> entitiesManager;
  entitiesManager.addPerceptionModule("No_ros", &test_perception);
  entitiesManager.addPerceptionModule("Ros", &test_ros_perception);
  entitiesManager.addPerceptionModule("Ros_sync", &test_ros_perception_sync);
  test_ros_perception_sync.activate(false);
  std::cout << "All modules: " << entitiesManager.getModulesListStr() << std::endl;
  std::cout << "Activated modules: " << entitiesManager.getActivatedModulesListStr() << std::endl;

  bool has_run = entitiesManager.update();
  std::cout << "has run = " << has_run << std::endl;

  test_perception.sendPerception({1,2,3});

  has_run = entitiesManager.update();
  std::cout << "has run = " << has_run << std::endl;

  ros::spin();

  return 0;
}