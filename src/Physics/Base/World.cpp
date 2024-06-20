#include "overworld/Physics/Base/World.h"

#include <cassert>
#include <urdf/model.h>

#include "overworld/Compat/ROS.h"
#include "overworld/Graphics/Base/ModelManager.h"
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
    return owds::ShapeCustomMesh{
      scale,
      owds::ModelManager::get().load(path)};
  }

  owds::Actor& World::createActor(const owds::Shape& shape)
  {
    return createActor(shape, {shape});
  }

  owds::Robot& World::loadRobotFromDescription(const std::string& path)
  {
    urdf::Model urdf_model;
    assert(urdf_model.initFile((base_assets_path_ / path).string()));

    auto robot = std::make_unique<owds::Robot>();
    const auto robot_ptr = robot.get();
    loaded_robots_[robot_ptr] = std::move(robot);

    robot_ptr->name_ = urdf_model.name_;

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

  void World::processLinks(owds::Robot& robot, const urdf::Link& link)
  {
    for(auto& child_link : link.child_links)
    {
      processLinks(robot, *child_link);
    }

    processLink(robot, link);
  }

  void World::processLink(owds::Robot& robot, const urdf::Link& urdf_link)
  {
    (void)robot;

    owds::Shape collision_shape = owds::ShapeDummy();
    std::vector<owds::Shape> visual_shapes;

    if(urdf_link.collision)
    {
      collision_shape = processShape(*urdf_link.collision->geometry);
    }

    assert(urdf_link.collision_array.size() <= 1 && "Links with multiple collision shapes are not supported at this moment.");

    visual_shapes.reserve(urdf_link.visual_array.size());

    for(auto& visual_geometry : urdf_link.visual_array)
    {
      visual_shapes.emplace_back(processShape(*visual_geometry->geometry));
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
    auto& link0 = *robot.links_.at(joint.parent_link_name);
    auto& link1 = *robot.links_.at(joint.child_link_name);

    // joint.parent_to_joint_origin_transform;
    switch(joint.type)
    {
    case urdf::Joint::REVOLUTE:
    {
      createJointRevolute(
        link0,
        {},
        {
          static_cast<float>(joint.parent_to_joint_origin_transform.rotation.x),
          static_cast<float>(joint.parent_to_joint_origin_transform.rotation.y),
          static_cast<float>(joint.parent_to_joint_origin_transform.rotation.z),
          static_cast<float>(joint.parent_to_joint_origin_transform.rotation.w),
        },
        link1,
        {},
        {});

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

  owds::Shape World::processShape(const urdf::Geometry& shape)
  {
    switch(shape.type)
    {
    case urdf::Geometry::SPHERE:
    {
      auto& sphere = dynamic_cast<const urdf::Sphere&>(shape);
      return createShapeSphere(static_cast<float>(sphere.radius));
    }
    case urdf::Geometry::BOX:
    {
      auto& box = dynamic_cast<const urdf::Box&>(shape);
      return createShapeBox({static_cast<float>(box.dim.x),
                             static_cast<float>(box.dim.y),
                             static_cast<float>(box.dim.z)});
    }
    case urdf::Geometry::CYLINDER:
    {
      // I hope the cylinder is ok
      auto& cylinder = dynamic_cast<const urdf::Cylinder&>(shape);
      return createShapeCylinder(static_cast<float>(cylinder.radius),
                                 static_cast<float>(cylinder.length));
    }
    case urdf::Geometry::MESH:
    {
      auto& mesh = dynamic_cast<const urdf::Mesh&>(shape);
      const auto url_handler = mesh.filename.substr(0, 10);

      assert(url_handler == "package://");

      const auto total_path = mesh.filename.substr(10);

      auto parent_path = std::filesystem::path(total_path);
      while(parent_path.has_parent_path())
      {
        parent_path = parent_path.parent_path();
      }

      const auto assets_path = compat::owds_ros::getShareDirectory(parent_path.string());
      const auto relative_path = total_path.substr(parent_path.string().size() + 1);

      return createShapeFromModel(assets_path + "/" + relative_path,
                                  {static_cast<float>(mesh.scale.x),
                                   static_cast<float>(mesh.scale.y),
                                   static_cast<float>(mesh.scale.z)});
    }
    default:
      assert(false && "Unrecognized/unsupported shape");
    }
  }
} // namespace owds