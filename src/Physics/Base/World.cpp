#include "overworld/Physics/Base/World.h"

#include <cassert>
#include <overworld/Physics/Base/World.h>
#include <urdf/model.h>

#include "overworld/Compat/ROS.h"
#include "overworld/Graphics/Base/Material.h"
#include "overworld/Graphics/Base/ModelManager.h"
#include "overworld/Helper/ROS.h"
#include "overworld/Physics/Base/Actor.h"
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

  owds::Shape World::createShapeBox(const owds::Color& color, const std::array<float, 3>& half_extents)
  {
    return owds::ShapeBox{
      half_extents,
      color,
      preloaded_box_model_};
  }

  owds::Shape World::createShapeCapsule(const owds::Color& color, const float radius, const float height)
  {
    return owds::ShapeCapsule{
      radius,
      height,
      color,
      preloaded_cylinder_model_,
      preloaded_sphere_model_};
  }

  owds::Shape World::createShapeCylinder(const owds::Color& color, const float radius, const float height)
  {
    return owds::ShapeCylinder{
      radius,
      height,
      color,
      preloaded_cylinder_model_};
  }

  owds::Shape World::createShapeSphere(const owds::Color& color, const float radius)
  {
    return owds::ShapeSphere{
      radius,
      color,
      preloaded_sphere_model_};
  }

  owds::Shape World::createShapeFromModel(const owds::Material& material, const std::string& path, const std::array<float, 3>& scale)
  {
    return owds::ShapeCustomMesh{
      scale,
      material,
      owds::ModelManager::get().load(path)};
  }

  owds::Robot& World::loadRobotFromDescription(const std::string& path)
  {
    urdf::Model urdf_model;
    assert(urdf_model.initFile((base_assets_path_ / path).string()));

    auto robot = std::make_unique<owds::Robot>();
    const auto robot_ptr = robot.get();
    loaded_robots_[robot_ptr] = std::move(robot);

    robot_ptr->name_ = urdf_model.name_;

    for(auto& [name, material] : urdf_model.materials_)
    {
      processMaterial(*robot_ptr, *material);
    }

    for(auto& [name, link] : urdf_model.links_)
    {
      processLinks(*robot_ptr, *link);
    }

    for(const auto& [name, joint] : urdf_model.joints_)
    {
      processJoint(*robot_ptr, *joint);
    }

    for(auto& [name, link] : urdf_model.links_)
    {
      processJoints(*robot_ptr, *link);
    }

    return *robot_ptr;
  }

  void World::processMaterial(owds::Robot& robot, const urdf::Material& urdf_material)
  {
    robot.materials_[urdf_material.name] = {
      !urdf_material.texture_filename.empty() ?
        owds::Color{255,                                                      255, 255, 255}
        :
        owds::Color{static_cast<std::uint8_t>(urdf_material.color.r * 255.f),
                    static_cast<std::uint8_t>(urdf_material.color.g * 255.f),
                    static_cast<std::uint8_t>(urdf_material.color.b * 255.f),
                    static_cast<std::uint8_t>(urdf_material.color.a * 255.f)               },
      urdf_material.texture_filename.empty() ?
        "" :
        owds::rosPkgPathToPath(urdf_material.texture_filename)
    };
  }

  void World::processLinks(owds::Robot& robot, const urdf::Link& urdf_link)
  {
    for(auto& child_link : urdf_link.child_links)
    {
      processLinks(robot, *child_link);
    }

    processLink(robot, urdf_link);
  }

  void World::processLink(owds::Robot& robot, const urdf::Link& urdf_link)
  {
    const auto material_name = urdf_link.visual ? urdf_link.visual->material_name : "";
    const auto material = material_name.empty() ? owds::Material{owds::Color{255, 255, 255, 255}, ""} : robot.materials_.at(material_name);

    owds::Shape collision_shape = owds::ShapeDummy();
    std::vector<owds::Shape> visual_shapes;

    if(urdf_link.collision)
    {
      collision_shape = convertShape(material, *urdf_link.collision->geometry);
    }

    assert(urdf_link.collision_array.size() <= 1 && "Links with multiple collision shapes are not supported at this moment.");

    visual_shapes.reserve(urdf_link.visual_array.size());

    for(auto& visual_geometry : urdf_link.visual_array)
    {
      visual_shapes.emplace_back(convertShape(material, *visual_geometry->geometry));
    }

    auto& link = createActor(collision_shape, visual_shapes);

    if(urdf_link.inertial)
    {
      const auto& inertial = *urdf_link.inertial;
      link.setPositionAndOrientation({static_cast<float>(inertial.origin.position.x),
                                      static_cast<float>(inertial.origin.position.y),
                                      static_cast<float>(inertial.origin.position.z)},
                                     {static_cast<float>(inertial.origin.rotation.x),
                                      static_cast<float>(inertial.origin.rotation.y),
                                      static_cast<float>(inertial.origin.rotation.z),
                                      static_cast<float>(inertial.origin.rotation.w)});
      link.setMass(static_cast<float>(inertial.mass));
    }

    robot.links_[urdf_link.name] = std::addressof(link);
  }

  void World::processJoints(owds::Robot& robot, const urdf::Link& urdf_link)
  {
    for(const auto& joint : urdf_link.child_joints)
    {
      processJoint(robot, *joint);
    }

    for(const auto& child_link : urdf_link.child_links)
    {
      processJoints(robot, *child_link);
    }
  }

  void World::processJoint(owds::Robot& robot, const urdf::Joint& urdf_joint)
  {
    (void) robot;
    // auto& link0 = *robot.links_.at(urdf_joint.parent_link_name);
    // auto& link1 = *robot.links_.at(urdf_joint.child_link_name);

    // joint.parent_to_joint_origin_transform;
    switch(urdf_joint.type)
    {
    case urdf::Joint::REVOLUTE:
    {
      /*createJointRevolute(
        link0,
        {},
        {
          static_cast<float>(urdf_joint.parent_to_joint_origin_transform.rotation.x),
          static_cast<float>(urdf_joint.parent_to_joint_origin_transform.rotation.y),
          static_cast<float>(urdf_joint.parent_to_joint_origin_transform.rotation.z),
          static_cast<float>(urdf_joint.parent_to_joint_origin_transform.rotation.w),
        },
        link1,
        {},
        {});*/

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

  owds::Shape World::convertShape(const owds::Material& material, const urdf::Geometry& urdf_shape)
  {
    switch(urdf_shape.type)
    {
    case urdf::Geometry::SPHERE:
    {
      auto& sphere = dynamic_cast<const urdf::Sphere&>(urdf_shape);
      return createShapeSphere(material.color_rgba_,
                               static_cast<float>(sphere.radius));
    }
    case urdf::Geometry::BOX:
    {
      auto& box = dynamic_cast<const urdf::Box&>(urdf_shape);
      return createShapeBox(material.color_rgba_,
                            {static_cast<float>(box.dim.x),
                             static_cast<float>(box.dim.y),
                             static_cast<float>(box.dim.z)});
    }
    case urdf::Geometry::CYLINDER:
    {
      // I hope the cylinder is ok
      auto& cylinder = dynamic_cast<const urdf::Cylinder&>(urdf_shape);
      return createShapeCylinder(material.color_rgba_,
                                 static_cast<float>(cylinder.radius),
                                 static_cast<float>(cylinder.length));
    }
    case urdf::Geometry::MESH:
    {
      auto& mesh = dynamic_cast<const urdf::Mesh&>(urdf_shape);

      return createShapeFromModel(material,
                                  owds::rosPkgPathToPath(mesh.filename),
                                  {static_cast<float>(mesh.scale.x),
                                   static_cast<float>(mesh.scale.y),
                                   static_cast<float>(mesh.scale.z)});
    }
    default:
      assert(false && "Unrecognized/unsupported shape");
    }
  }
} // namespace owds