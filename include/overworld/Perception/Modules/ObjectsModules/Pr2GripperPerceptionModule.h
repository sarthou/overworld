#ifndef OWDS_PR2GRIPPERPERCEPTIONMODULE_H
#define OWDS_PR2GRIPPERPERCEPTIONMODULE_H

#include "overworld/BasicTypes/Object.h"
#include "overworld/BasicTypes/Agent.h"
#include "overworld/Bullet/BulletClient.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"

#include "pr2_msgs/PressureState.h"

namespace owds {

enum Pr2GripperSide_e
{
  PR2_GRIPPER_LEFT,
  PR2_GRIPPER_RIGHT,
  PR2_GRIPER_UNDEFINED
};

class Pr2GripperPerceptionModule : public PerceptionModuleRosBase<Object, pr2_msgs::PressureState>
{
public:
  Pr2GripperPerceptionModule();

  void setParameter(const std::string& parameter_name, const std::string& parameter_value);
  bool closeInitialization();

private:
  Pr2GripperSide_e side_;
  int pr2_left_tip_bullet_id_;
  int pr2_right_tip_bullet_id_;

  bool is_init_;
  double left_tip_pressure_;
  double left_tip_pressure_prev_;
  double right_tip_pressure_;
  double right_tip_pressure_prev_;

  bool has_picked_;
  size_t obj_id_;
  std::string current_obj_id_;

  double pressure_threshold_;
  double distance_threshold_;
  double min_period_;
  ros::Time last_update_;

  bool perceptionCallback(const pr2_msgs::PressureState& msg);
  double getGripperDistance();
};

} // namespace owds

#endif // OWDS_PR2GRIPPERPERCEPTIONMODULE_H