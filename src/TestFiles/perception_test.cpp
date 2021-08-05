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
  bool perceptionCallback(const std::vector<int>& msg)
  {
    std::cout << "PerceptionTest: ";
    for(auto i : msg)
      std::cout << i << " ";
    std::cout << std::endl;
    return true;
  }
};

class RosPerceptionTest : public owds::PerceptionModuleRosBase<owds::Entity, std_msgs::String>
{
public:
  RosPerceptionTest(ros::NodeHandle* n) : PerceptionModuleRosBase(n, "owds/pecpetion_test_str") {}
  virtual ~RosPerceptionTest() = default;

private:
  bool perceptionCallback(const std_msgs::String& msg)
  {
    std::cout << "RosPerceptionTest: " << msg.data << std::endl;
    return true;
  }
};

class RosPerceptionSyncTest : public owds::PerceptionModuleRosSyncBase<owds::Entity, overworld::StampedStringTest, overworld::StampedStringTest>
{
public:
  RosPerceptionSyncTest(ros::NodeHandle* n) : PerceptionModuleRosSyncBase(n, "owds/sync/pecpetion_test_str_0", "owds/sync/pecpetion_test_str_1") {}
  virtual ~RosPerceptionSyncTest() = default;

private:
  bool perceptionCallback(const overworld::StampedStringTest& msg_1, const overworld::StampedStringTest& msg_2)
  {
    std::cout << "RosPerceptionSyncTest: " << msg_1.data << " : " << msg_2.data << std::endl;
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  PerceptionTest test_perception;
  RosPerceptionTest test_ros_perception(&n);
  RosPerceptionSyncTest test_ros_perception_sync(&n);

  owds::EntitiesPerceptionManager<owds::Entity> entities_manager;
  entities_manager.addPerceptionModule("No_ros", &test_perception);
  entities_manager.addPerceptionModule("Ros", &test_ros_perception);
  entities_manager.addPerceptionModule("Ros_sync", &test_ros_perception_sync);
  test_ros_perception_sync.activate(false);
  std::cout << "All modules: " << entities_manager.getModulesListStr() << std::endl;
  std::cout << "Activated modules: " << entities_manager.getActivatedModulesListStr() << std::endl;

  bool has_run = entities_manager.update();
  std::cout << "has run = " << has_run << std::endl;

  test_perception.sendPerception({1,2,3});

  has_run = entities_manager.update();
  std::cout << "has run = " << has_run << std::endl;

  ros::spin();

  return 0;
}