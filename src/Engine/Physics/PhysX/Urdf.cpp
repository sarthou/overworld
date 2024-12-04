#include "overworld/Engine/Physics/PhysX/Urdf.h"

#include <PxArticulationLink.h>
#include <array>
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
#include "overworld/Engine/Common/Urdf/Urdf.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"
#include "overworld/Engine/Physics/PhysX/Actors/LinkActor.h"
#include "overworld/Engine/Physics/PhysX/Context.h"
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
    ctx_.px_scene_->addArticulation(*px_articulation_);
    px_articulation_->setArticulationFlag(::physx::PxArticulationFlag::eDISABLE_SELF_COLLISION, true);

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
    LinkActor* link_actor = new LinkActor(ctx_, px_link, collision_shape, visual_shapes);
    link_actor->setup({0., 0., 0.}, {0., 0., 0., 1.}); // position is not used for link actors

    px_links_[link_name] = px_link;
    links_[link_name] = link_actor;
  }

  void Urdf::addJoint(const urdf::Joint_t& joint)
  {
    auto* px_link = px_links_[joint.child_link];
    ::physx::PxArticulationJointReducedCoordinate* px_joint = static_cast<::physx::PxArticulationJointReducedCoordinate*>(px_link->getInboundJoint());
    switch(joint.type)
    {
    case urdf::JointType_e::joint_revolute:
      px_joint->setJointType(::physx::PxArticulationJointType::eREVOLUTE);
      if(joint.axis[0] != 0.)
        px_joint->setMotion(::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationMotion::eFREE); // Unbounded rotation around twist axis
      else if(joint.axis[1] != 0.)
        px_joint->setMotion(::physx::PxArticulationAxis::eSWING1, ::physx::PxArticulationMotion::eFREE); // Unbounded rotation around swing1 axis
      else if(joint.axis[2] != 0.)
        px_joint->setMotion(::physx::PxArticulationAxis::eSWING2, ::physx::PxArticulationMotion::eFREE); // Unbounded rotation around swing2 axis
      else
        std::cout << "-----------> no axis set" << std::endl;
      // px_joint->setMotion(::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationMotion::eLIMITED); // Limited rotation around the twist axis
      // px_joint->setLimit(::physx::PxArticulationAxis::eTWIST, -::physx::PxPi / 4, ::physx::PxPi / 4);    // ±45 degrees
      break;
    case urdf::JointType_e::joint_continuous:
      px_joint->setJointType(::physx::PxArticulationJointType::eREVOLUTE);
      if(joint.axis[0] != 0.)
        px_joint->setMotion(::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationMotion::eFREE); // Unbounded rotation around twist axis
      else if(joint.axis[1] != 0.)
        px_joint->setMotion(::physx::PxArticulationAxis::eSWING1, ::physx::PxArticulationMotion::eFREE); // Unbounded rotation around swing1 axis
      else if(joint.axis[2] != 0.)
        px_joint->setMotion(::physx::PxArticulationAxis::eSWING2, ::physx::PxArticulationMotion::eFREE); // Unbounded rotation around swing2 axis
      else
        std::cout << "-----------> no axis set" << std::endl;
      break;
    case urdf::JointType_e::joint_prismatic:
      px_joint->setJointType(::physx::PxArticulationJointType::ePRISMATIC);
      if(joint.axis[0] != 0.)
        px_joint->setMotion(::physx::PxArticulationAxis::eX, ::physx::PxArticulationMotion::eFREE); // Unbounded rotation around X axis
      else if(joint.axis[1] != 0.)
        px_joint->setMotion(::physx::PxArticulationAxis::eY, ::physx::PxArticulationMotion::eFREE); // Unbounded rotation around Y axis
      else if(joint.axis[2] != 0.)
        px_joint->setMotion(::physx::PxArticulationAxis::eZ, ::physx::PxArticulationMotion::eFREE); // Unbounded rotation around Z axis
      else
        std::cout << "-----------> no axis set" << std::endl;
      // px_joint->setMotion(::physx::PxArticulationAxis::eX, ::physx::PxArticulationMotion::eLIMITED); // Limited translation along X-axis
      // px_joint->setLimit(::physx::PxArticulationAxis::eX, 0.0f, 1.0f);                               // Translation range: 0 to 1 unit
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
  }

  void Urdf::remove()
  {
    ctx_.px_scene_->removeArticulation(*px_articulation_);
  }

  void Urdf::setPhysicsEnabled(bool enabled)
  {
    for(const auto& link : px_links_)
      link.second->setActorFlag(::physx::PxActorFlag::eDISABLE_GRAVITY, !enabled);
  }

} // namespace owds::physx