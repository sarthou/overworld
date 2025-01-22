#include "overworld/Engine/Physics/PhysX/Urdf.h"

#include <PxArticulationLink.h>
#include <array>
#include <cstddef>
#include <foundation/PxQuat.h>
#include <foundation/PxSimpleTypes.h>
#include <foundation/PxTransform.h>
#include <foundation/PxVec3.h>
#include <glm/detail/type_quat.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <iostream>
#include <overworld/Utils/BitCast.h>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Urdf/Joint.h"
#include "overworld/Engine/Common/Urdf/Urdf.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"
#include "overworld/Engine/Physics/PhysX/Actors/LinkActor.h"
#include "overworld/Engine/Physics/PhysX/Context.h"
#include "overworld/Engine/Physics/PhysX/Joint.h"
#include "overworld/Engine/Physics/PhysX/SharedContext.h"
#include "solver/PxSolverDefs.h"

namespace owds::physx {

  Urdf::Urdf(owds::physx::Context& ctx) : ctx_(ctx)
  {}

  void Urdf::setup()
  {
    const auto& sdk = owds::physx::Context::createContext()->px_physics_;

    px_articulation_ = sdk->createArticulationReducedCoordinate();
  }

  void Urdf::finish()
  {
    ctx_.physx_mutex_.lock();
    ctx_.px_scene_->addArticulation(*px_articulation_);
    px_articulation_->setArticulationFlag(::physx::PxArticulationFlag::eDISABLE_SELF_COLLISION, true);
    ctx_.physx_mutex_.unlock();

    setPhysicsEnabled(false);
  }

  void Urdf::addLink(const std::string& parent,
                     const std::string& link_name,
                     const glm::vec3& origin_translation,
                     const glm::vec3& origin_rotation,
                     const owds::Shape& collision_shape,
                     const std::vector<owds::Shape>& visual_shapes)
  {
    ::physx::PxArticulationLink* parent_ptr = nullptr;
    if(parent.empty() == false)
      parent_ptr = px_links_[parent];

    glm::quat rot(origin_rotation);

    const auto px_transform =
      ::physx::PxTransform(
        ::physx::PxVec3(
          static_cast<::physx::PxReal>(origin_translation.x),
          static_cast<::physx::PxReal>(origin_translation.y),
          static_cast<::physx::PxReal>(origin_translation.z)),
        ::physx::PxQuat(
          static_cast<::physx::PxReal>(rot.x),
          static_cast<::physx::PxReal>(rot.y),
          static_cast<::physx::PxReal>(rot.z),
          static_cast<::physx::PxReal>(rot.w)));

    ::physx::PxArticulationLink* px_link = px_articulation_->createLink(parent_ptr, px_transform);
    LinkActor* link_actor = new LinkActor(ctx_, px_link, unique_id_, collision_shape, visual_shapes);
    link_actor->setup({0., 0., 0.}, {0., 0., 0., 1.}); // position is not used for link actors

    px_links_[link_name] = px_link;
    links_[link_name] = link_actor;
    id_links_[link_actor->unique_id_] = link_actor;
    if(parent_ptr == nullptr)
      root_actor_ = link_actor;
  }

  void Urdf::addJoint(const urdf::Joint_t& joint)
  {
    ctx_.physx_mutex_.lock();
    auto* px_link = px_links_[joint.child_link];
    ::physx::PxArticulationJointReducedCoordinate* px_joint = static_cast<::physx::PxArticulationJointReducedCoordinate*>(px_link->getInboundJoint());
    px_joints_[joint.name] = px_joint;

    ::physx::PxArticulationAxis::Enum axis(::physx::PxArticulationAxis::eCOUNT);
    int direction = joint.axis[0] + joint.axis[1] + joint.axis[2];
    switch(joint.type)
    {
    case urdf::JointType_e::joint_revolute:
      px_joint->setJointType(::physx::PxArticulationJointType::eREVOLUTE);
      if(joint.axis[0] != 0.)
        axis = ::physx::PxArticulationAxis::eTWIST;
      else if(joint.axis[1] != 0.)
        axis = ::physx::PxArticulationAxis::eSWING1;
      else if(joint.axis[2] != 0.)
        axis = ::physx::PxArticulationAxis::eSWING2;

      if(axis != ::physx::PxArticulationAxis::eCOUNT)
      {
        if(joint.limited)
        {
          px_joint->setMotion(axis, ::physx::PxArticulationMotion::eLIMITED);
          ::physx::PxArticulationLimit limit;
          limit.low = joint.limit.first;
          limit.high = joint.limit.second;
          px_joint->setLimitParams(axis, limit);
        }
        else
          px_joint->setMotion(axis, ::physx::PxArticulationMotion::eFREE);
      }

      break;

    case urdf::JointType_e::joint_continuous:
      px_joint->setJointType(::physx::PxArticulationJointType::eREVOLUTE);
      if(joint.axis[0] != 0.)
        axis = ::physx::PxArticulationAxis::eTWIST;
      else if(joint.axis[1] != 0.)
        axis = ::physx::PxArticulationAxis::eSWING1;
      else if(joint.axis[2] != 0.)
        axis = ::physx::PxArticulationAxis::eSWING2;

      if(axis != ::physx::PxArticulationAxis::eCOUNT)
        px_joint->setMotion(axis, ::physx::PxArticulationMotion::eFREE); // Unbounded rotation
      break;

    case urdf::JointType_e::joint_prismatic:
      px_joint->setJointType(::physx::PxArticulationJointType::ePRISMATIC);
      if(joint.axis[0] != 0.)
        axis = ::physx::PxArticulationAxis::eX;
      else if(joint.axis[1] != 0.)
        axis = ::physx::PxArticulationAxis::eY;
      else if(joint.axis[2] != 0.)
        axis = ::physx::PxArticulationAxis::eZ;

      if(axis != ::physx::PxArticulationAxis::eCOUNT)
      {
        if(joint.limited)
        {
          px_joint->setMotion(axis, ::physx::PxArticulationMotion::eLIMITED);
          ::physx::PxArticulationLimit limit;
          limit.low = joint.limit.first;
          limit.high = joint.limit.second;
          px_joint->setLimitParams(axis, limit);
        }
        else
          px_joint->setMotion(axis, ::physx::PxArticulationMotion::eFREE);
      }
      break;

    case urdf::JointType_e::joint_fixed:
      px_joint->setJointType(::physx::PxArticulationJointType::eFIX);
      break;

    case urdf::JointType_e::joint_floating:
      px_joint->setJointType(::physx::PxArticulationJointType::eUNDEFINED); // Free 6-DOF motion
      px_joint->setMotion(::physx::PxArticulationAxis::eX, ::physx::PxArticulationMotion::eFREE);
      px_joint->setMotion(::physx::PxArticulationAxis::eY, ::physx::PxArticulationMotion::eFREE);
      px_joint->setMotion(::physx::PxArticulationAxis::eZ, ::physx::PxArticulationMotion::eFREE);
      px_joint->setMotion(::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationMotion::eFREE);
      px_joint->setMotion(::physx::PxArticulationAxis::eSWING1, ::physx::PxArticulationMotion::eFREE);
      px_joint->setMotion(::physx::PxArticulationAxis::eSWING2, ::physx::PxArticulationMotion::eFREE);
      break;

    case urdf::JointType_e::joint_planar:
      // TODO consider joint axis
      px_joint->setJointType(::physx::PxArticulationJointType::eUNDEFINED);                           // Planar motion needs custom configuration
      px_joint->setMotion(::physx::PxArticulationAxis::eX, ::physx::PxArticulationMotion::eFREE);     // Free translation along X
      px_joint->setMotion(::physx::PxArticulationAxis::eZ, ::physx::PxArticulationMotion::eFREE);     // Free translation along Z
      px_joint->setMotion(::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationMotion::eFREE); // Free rotation around Y (plane's normal)
      break;

    default:
      break;
    }

    if(px_joint != nullptr)
    {
      int int_axis = -1;
      if((axis == ::physx::PxArticulationAxis::eX) || (axis == ::physx::PxArticulationAxis::eTWIST))
        int_axis = 0;
      else if((axis == ::physx::PxArticulationAxis::eY) || (axis == ::physx::PxArticulationAxis::eSWING1))
        int_axis = 1;
      else if((axis == ::physx::PxArticulationAxis::eZ) || (axis == ::physx::PxArticulationAxis::eSWING2))
        int_axis = 2;

      joints_[joint.name] = new ::owds::physx::Joint(ctx_, px_joint, joint.type, int_axis, direction);
    }
    ctx_.physx_mutex_.unlock();
  }

  void Urdf::remove()
  {
    ctx_.physx_mutex_.lock();
    ctx_.px_scene_->removeArticulation(*px_articulation_);
    ctx_.physx_mutex_.unlock();
  }

  void Urdf::setPhysicsEnabled(bool enabled)
  {
    ctx_.physx_mutex_.lock();
    for(const auto& link : px_links_)
      link.second->setActorFlag(::physx::PxActorFlag::eDISABLE_GRAVITY, !enabled);
    ctx_.physx_mutex_.unlock();
  }

  size_t Urdf::getNumJoints()
  {
    return px_joints_.size();
  }

  std::pair<std::array<double, 3>, std::array<double, 4>> Urdf::getPositionAndOrientation()
  {
    return root_actor_->getPositionAndOrientation();
  }

  void Urdf::setPositionAndOrientation(const std::array<double, 3>& position, const std::array<double, 4>& orientation)
  {
    const auto px_transform =
      ::physx::PxTransform(
        ::physx::PxVec3(
          static_cast<::physx::PxReal>(position[0]),
          static_cast<::physx::PxReal>(position[1]),
          static_cast<::physx::PxReal>(position[2])),
        ::physx::PxQuat(
          static_cast<::physx::PxReal>(orientation[0]),
          static_cast<::physx::PxReal>(orientation[1]),
          static_cast<::physx::PxReal>(orientation[2]),
          static_cast<::physx::PxReal>(orientation[3])));

    ctx_.physx_mutex_.lock();
    px_articulation_->setRootGlobalPose(px_transform);
    ctx_.physx_mutex_.unlock();
  }

  void Urdf::setVelocity(const std::array<double, 3>& linear_velocity, const std::array<double, 3>& angular_velocity)
  {
    ctx_.physx_mutex_.lock();
    px_articulation_->setRootLinearVelocity(::physx::PxVec3(
      static_cast<::physx::PxReal>(linear_velocity[0]),
      static_cast<::physx::PxReal>(linear_velocity[1]),
      static_cast<::physx::PxReal>(linear_velocity[2])));

    px_articulation_->setRootAngularVelocity(::physx::PxVec3(
      static_cast<::physx::PxReal>(angular_velocity[0]),
      static_cast<::physx::PxReal>(angular_velocity[1]),
      static_cast<::physx::PxReal>(angular_velocity[2])));
    ctx_.physx_mutex_.unlock();
  }

} // namespace owds::physx