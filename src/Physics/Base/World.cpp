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

  World::~World() = default;

  owds::Shape World::createShapeBox(const std::array<float, 3>& half_extents)
  {
    return owds::ShapeBox{
      half_extents,
      preloaded_box_model_};
  }

  owds::Shape World::createShapeCapsule(const float radius, const float height)
  {
    return owds::ShapeCapsule{
      radius,
      height,
      preloaded_cylinder_model_,
      preloaded_sphere_model_};
  }

  owds::Shape World::createShapeCylinder(const float radius, const float height)
  {
    return owds::ShapeCylinder{
      radius,
      height,
      preloaded_cylinder_model_};
  }

  owds::Shape World::createShapeSphere(const float radius)
  {
    return owds::ShapeSphere{
      radius,
      preloaded_sphere_model_};
  }

  owds::Shape World::createShapeFromModel(const std::string& path, const std::array<float, 3>& scale)
  {
    // todo: load model and return shape with model ref
    (void) path;
    (void) scale;
    assert(false && "not implemented");
  }

  owds::Actor& World::createActor(const owds::Shape& shape)
  {
    return createActor(shape, {shape});
  }

  owds::Robot& World::loadRobotFromDescription(const std::string& path)
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

    assert(false && "not implemented");
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
    (void) robot;

    if(link.visual)
    {
    }

    if(link.collision)
    {
    }

    assert(link.collision_array.size() <= 1 && "Links with multiple collision shapes are not supported at this moment.");

    for(auto& visual_geometry : link.visual_array)
    {
      (void) visual_geometry;
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
    (void) robot;

    switch(joint.type)
    {
    case urdf::Joint::REVOLUTE:
    {
      break;
    }
    case urdf::Joint::CONTINUOUS:
    {
      break;
    }
    case urdf::Joint::PRISMATIC:
    {
      break;
    }
    case urdf::Joint::FLOATING:
    {
      break;
    }
    case urdf::Joint::PLANAR:
    {
      break;
    }
    case urdf::Joint::FIXED:
    {
      break;
    }
    case urdf::Joint::UNKNOWN:
    default:
    {
      assert(false && "Unrecognized/unsupported joint type.");
    }
    }
  }
} // namespace owds