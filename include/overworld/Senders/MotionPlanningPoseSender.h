#ifndef OWDS_MOTIONPLANNINGPOSESENDER_H
#define OWDS_MOTIONPLANNINGPOSESENDER_H

#include "overworld/Perception/Managers/ObjectsPerceptionManager.h"

#include <ros/service_server.h>
#include <pr2_motion_tasks_msgs/GetPose.h>

namespace owds
{

class MotionPlanningPoseSender{
public:
    MotionPlanningPoseSender(ros::NodeHandle* nh_, ObjectsPerceptionManager& object_perception_manager);

    bool onGetPoseService(pr2_motion_tasks_msgs::GetPose::Request& req, pr2_motion_tasks_msgs::GetPose::Response& res);
protected:
ros::ServiceServer get_pose_service_;
ObjectsPerceptionManager& object_perception_manager_;

};

} // namespace owds

#endif /* OWDS_MOTIONPLANNINGPOSESENDER_H */
