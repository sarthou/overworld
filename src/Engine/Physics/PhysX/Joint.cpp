#include "overworld/Engine/Physics/PhysX/Joint.h"

#include <PxArticulationJointReducedCoordinate.h>
#include <cassert>

#include "overworld/Engine/Common/Urdf/Joint.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"
#include "overworld/Engine/Physics/PhysX/Context.h"
#include "solver/PxSolverDefs.h"

namespace owds::physx {

  Joint::Joint(owds::physx::Context& ctx,
               ::physx::PxArticulationJointReducedCoordinate* px_joint,
               urdf::JointType_e type, int axis) : owds::Joint(type, axis),
                                                   ctx_(ctx),
                                                   px_axis_(::physx::PxArticulationAxis::eCOUNT),
                                                   px_base_(px_joint)
  {
    switch(type_)
    {
    case urdf::JointType_e::joint_revolute:
    case urdf::JointType_e::joint_continuous:
      if(axis_ == 0)
        px_axis_ = ::physx::PxArticulationAxis::eTWIST;
      else if(axis_ == 1)
        px_axis_ = ::physx::PxArticulationAxis::eSWING1;
      else if(axis_ == 2)
        px_axis_ = ::physx::PxArticulationAxis::eSWING2;
      break;

    case urdf::JointType_e::joint_prismatic:
      if(axis_ == 0)
        px_axis_ = ::physx::PxArticulationAxis::eX;
      else if(axis_ == 1)
        px_axis_ = ::physx::PxArticulationAxis::eY;
      else if(axis_ == 2)
        px_axis_ = ::physx::PxArticulationAxis::eZ;

    default:
      break;
    }
  }

  void Joint::setPosition(double position)
  {
    ctx_.physx_mutex_.lock();
    assert(type_ != urdf::JointType_e::joint_fixed && "[Joint] fixed joint cannot be moved");
    if(px_axis_ != ::physx::PxArticulationAxis::eCOUNT)
      px_base_->setJointPosition(px_axis_, position);
    else
      assert(false && "[Joint floating and planar joints are not yet supported]");
    ctx_.physx_mutex_.unlock();
  }

  void Joint::setVelocity(double velocity)
  {
    ctx_.physx_mutex_.lock();
    assert(type_ != urdf::JointType_e::joint_fixed && "[Joint] fixed joint cannot be moved");
    if(px_axis_ != ::physx::PxArticulationAxis::eCOUNT)
      px_base_->setJointVelocity(px_axis_, velocity);
    else
      assert(false && "[Joint floating and planar joints are not yet supported]");
    ctx_.physx_mutex_.unlock();
  }

} // namespace owds::physx