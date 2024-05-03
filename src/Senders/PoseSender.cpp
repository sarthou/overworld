#include "overworld/Senders/PoseSender.h"

#include <geometry_msgs/PoseStamped.h>

namespace owds {

PoseSender::PoseSender(ros::NodeHandle* nh_, ObjectsPerceptionManager& object_perception_manager)
  : object_perception_manager_(object_perception_manager)
{
  get_pose_service_ = nh_->advertiseService("/overworld/getPose", &PoseSender::onGetPoseService, this);
}

bool PoseSender::onGetPoseService(overworld::GetPose::Request& req, overworld::GetPose::Response& res)
{
  const auto objects = object_perception_manager_.getEntities();
  for (const std::string& id : req.ids)
  {
    geometry_msgs::PoseStamped pose;
    if (objects.find(id) == objects.end())
      pose.header.frame_id = "";
    else
    {
      const Object* obj = objects.at(id);
      if (obj->isLocated() == false)
        pose.header.frame_id = "";
      else
      {
        pose.pose = obj->pose().toPoseMsg();
        pose.header.stamp = obj->lastStamp();
        pose.header.frame_id = "map";
      }
    }
    res.poses.push_back(pose);
  }
  return true;
}

} // namespace owds
