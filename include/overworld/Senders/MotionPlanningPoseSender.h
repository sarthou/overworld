#ifndef OWDS_MOTIONPLANNINGPOSESENDER_H
#define OWDS_MOTIONPLANNINGPOSESENDER_H

#include "overworld/Perception/Managers/ObjectsPerceptionManager.h"

#include <ros/service_server.h>
#include <overworld/GetPose.h>

namespace owds
{

class MotionPlanningPoseSender{
public:
    MotionPlanningPoseSender(ros::NodeHandle* nh_, ObjectsPerceptionManager& object_perception_manager);

    bool onGetPoseService(overworld::GetPose::Request& req, overworld::GetPose::Response& res);
protected:
ros::ServiceServer get_pose_service_;
ObjectsPerceptionManager& object_perception_manager_;

};

} // namespace owds

#endif /* OWDS_MOTIONPLANNINGPOSESENDER_H */
