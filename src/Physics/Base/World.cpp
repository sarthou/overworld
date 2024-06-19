#include "overworld/Physics/Base/World.h"

#include <cassert>
#include <urdf/model.h>

#include "overworld/Graphics/Base/ModelManager.h"
#include "overworld/Physics/Base/Robot.h"

namespace owds {
  World::World(const std::filesystem::path& base_assets_path)
    : base_assets_path_(base_assets_path),
      preloaded_box_model_(owds::ModelManager::get().load(
        base_assets_path / "models/basic_shapes/cube.obj")),
      preloaded_cylinder_model_(owds::ModelManager::get().load(
        base_assets_path / "models/basic_shapes/cylinder.obj")),
      preloaded_sphere_model_(owds::ModelManager::get().load(
        base_assets_path / "models/basic_shapes/sphere.obj"))
  {}

  owds::Shape World::createShapeBox(const std::array<float, 3>& half_extents)
  {
    return owds::ShapeBox{
      .half_extents_ = half_extents,
      .box_model_ = preloaded_box_model_};
  }

  owds::Shape World::createShapeCapsule(const float radius, const float height)
  {
    return owds::ShapeCapsule{
      .radius_ = radius,
      .height_ = height,
      .cylinder_model_ = preloaded_cylinder_model_,
      .sphere_model_ = preloaded_sphere_model_};
  }

  owds::Shape World::createShapeCylinder(const float radius, const float height)
  {
    return owds::ShapeCylinder{
      .radius_ = radius,
      .height_ = height,
      .cylinder_model_ = preloaded_cylinder_model_};
  }

  owds::Shape World::createShapeSphere(const float radius)
  {
    return owds::ShapeSphere{
      .radius_ = radius,
      .sphere_model_ = preloaded_sphere_model_};
  }

  owds::Shape World::createShapeFromModel(const std::string& path, const std::array<float, 3>& scale)
  {
    // todo: load model and return shape with model ref
  }

  owds::Actor& World::createActor(const owds::Shape& shape)
  {
    return createActor(shape, { shape });
  }

  owds::Robot& World::loadRobot(const std::string& path)
  {
    urdf::Model urdf_model;
    assert(urdf_model.initFile((base_assets_path_ / path).string()));

    owds::Robot robot;
    robot.name_ = urdf_model.name_;

    for(auto& [name, link] : urdf_model.links_)
    {
      processLinks(robot, *link);
    }

    for(const auto& [name, joint] : urdf_model.joints_)
    {
      processJoint(robot, *joint);
    }

    for(auto& [name, link] : urdf_model.links_)
    {
      processJoints(robot, *link);
    }
  }

  void World::processLinks(owds::Robot& robot, const urdf::Link& link)
  {
    for(auto& child_link : link.child_links)
    {
      processLinks(robot, *child_link);
    }

    processLink(robot, link);
  }

  void World::processLink(owds::Robot& robot, const urdf::Link& link)
  {
    if(link.visual)
    {
    }

    if(link.collision)
    {
    }

    for(auto& collision_geometry : link.collision_array)
    {
    }

    for(auto& visual_geometry : link.visual_array)
    {
    }
  }

  void World::processJoints(owds::Robot& robot, const urdf::Link& link)
  {
    for(const auto& joint : link.child_joints)
    {
      processJoint(robot, *joint);
    }

    for(const auto& child_link : link.child_links)
    {
      processJoints(robot, *child_link);
    }
  }

  void World::processJoint(owds::Robot& robot, const urdf::Joint& joint)
  {
    switch(joint.type)
    {
    case urdf::Joint::REVOLUTE:
    {
    }
    case urdf::Joint::CONTINUOUS:
    {
    }
    case urdf::Joint::PRISMATIC:
    {
    }
    case urdf::Joint::FLOATING:
    {
    }
    case urdf::Joint::PLANAR:
    {
    }
    case urdf::Joint::FIXED:
    {
    }
    case urdf::Joint::UNKNOWN:
    default:
    {
      assert(false && "Unrecognized/unsupported joint type.");
    }
    }
  }
} // namespace owds