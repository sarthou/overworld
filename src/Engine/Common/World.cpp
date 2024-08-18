#include "overworld/Engine/Common/World.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <string>
#include <urdf/model.h>

#include "overworld/Compat/ROS.h"
#include "overworld/Engine/Common/Models/Color.h"
#include "overworld/Engine/Common/Models/Material.h"
#include "overworld/Engine/Common/Models/ModelManager.h"
#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Common/Urdf/Urdf.h"
#include "overworld/Utils/GlmMath.h"
#include "overworld/Utils/ROS.h"

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

  owds::Shape World::createShapeBox(const owds::Color& color, const std::array<float, 3>& half_extents, glm::mat4& transform)
  {
    return owds::ShapeBox{
      half_extents,
      color,
      preloaded_box_model_,
      transform};
  }

  owds::Shape World::createShapeCapsule(const owds::Color& color, const float radius, const float height, glm::mat4& transform)
  {
    return owds::ShapeCapsule{
      radius,
      height,
      color,
      preloaded_cylinder_model_,
      preloaded_sphere_model_,
      transform};
  }

  owds::Shape World::createShapeCylinder(const owds::Color& color, const float radius, const float height, glm::mat4& transform)
  {
    return owds::ShapeCylinder{
      radius,
      height,
      color,
      preloaded_cylinder_model_,
      transform};
  }

  owds::Shape World::createShapeSphere(const owds::Color& color, const float radius, glm::mat4& transform)
  {
    return owds::ShapeSphere{
      radius,
      color,
      preloaded_sphere_model_,
      transform};
  }

  owds::Shape World::createShapeFromModel(const owds::Material& material, const std::string& path, const std::array<float, 3>& scale, glm::mat4& transform)
  {
    return owds::ShapeCustomMesh{
      glm::vec3(scale[0], scale[1], scale[2]),
      material,
      owds::ModelManager::get().load(path),
      transform};
  }

  owds::Urdf& World::loadRobotFromDescription(const std::string& path, bool from_base_path)
  {
    urdf::Model urdf_model;
    if(from_base_path)
      assert(urdf_model.initFile((base_assets_path_ / path).string()));
    else
      assert(urdf_model.initFile(path));

    auto urdf = std::make_unique<owds::Urdf>();
    const auto urdf_ptr = urdf.get();
    loaded_urdfs_[urdf_ptr] = std::move(urdf);

    urdf_ptr->name_ = urdf_model.name_;

    for(auto& [name, material] : urdf_model.materials_)
    {
      processMaterial(*urdf_ptr, *material);
    }

    for(auto& [name, link] : urdf_model.links_)
    {
      processLinks(*urdf_ptr, *link);
    }

    for(const auto& [name, joint] : urdf_model.joints_)
    {
      processJoint(*urdf_ptr, *joint);
    }

    for(auto& [name, link] : urdf_model.links_)
    {
      processJoints(*urdf_ptr, *link);
    }

    return *urdf_ptr;
  }

  void World::processMaterial(owds::Urdf& robot, const urdf::Material& urdf_material)
  {
    robot.materials_[urdf_material.name] = {
      urdf_material.name,
      !urdf_material.texture_filename.empty() ?
        owds::Color{1,                     1, 1, 0.}
        :
        owds::Color{urdf_material.color.r,
                    urdf_material.color.g,
                    urdf_material.color.b,
                    urdf_material.color.a          },
      owds::Color{1,                     1, 1, 0.}, // specular
      -1.0, // shininess_
      urdf_material.texture_filename.empty() ?
        "" :
        owds::rosPkgPathToPath(urdf_material.texture_filename),
      "", // specular
      "", // normal
    };
  }

  void World::processLinks(owds::Urdf& robot, const urdf::Link& urdf_link)
  {
    for(auto& child_link : urdf_link.child_links)
    {
      processLinks(robot, *child_link);
    }

    processLink(robot, urdf_link);
  }

  void World::processLink(owds::Urdf& robot, const urdf::Link& urdf_link)
  {
    const auto material_name = urdf_link.visual ? urdf_link.visual->material_name : "";
    const auto material = material_name.empty() ? owds::Material{
                                                    "",
                                                    owds::Color{1, 1, 1, 0.},
                                                    owds::Color{1, 1, 1, 0.},
                                                    -1.0,
                                                    "", "", ""
    } :
                                                  robot.materials_.at(material_name);

    owds::Shape collision_shape = owds::ShapeDummy();
    std::vector<owds::Shape> visual_shapes;

    if(urdf_link.collision)
    {
      auto& origin = urdf_link.collision->origin;
      glm::vec3 position = ToV3({(float)origin.position.x, (float)origin.position.y, (float)origin.position.z});
      glm::quat rotation = ToQT({(float)origin.rotation.x, (float)origin.rotation.y, (float)origin.rotation.z, (float)origin.rotation.w});
      glm::mat4 translate = glm::translate(glm::mat4(1), position);
      glm::mat4 rotate = glm::mat4_cast(rotation);
      glm::mat4 transform = translate * rotate;
      collision_shape = convertShape(material, *urdf_link.collision->geometry, transform); // TODO
    }

    assert(urdf_link.collision_array.size() <= 1 && "Links with multiple collision shapes are not supported at this moment.");

    visual_shapes.reserve(urdf_link.visual_array.size());

    for(auto& visual_geometry : urdf_link.visual_array)
    {
      auto& origin = visual_geometry->origin;
      glm::vec3 position = ToV3({(float)origin.position.x, (float)origin.position.y, (float)origin.position.z});
      glm::quat rotation = ToQT({(float)origin.rotation.x, (float)origin.rotation.y, (float)origin.rotation.z, (float)origin.rotation.w});
      glm::mat4 translate = glm::translate(glm::mat4(1), position);
      glm::mat4 rotate = glm::mat4_cast(rotation);
      glm::mat4 transform = translate * rotate;
      visual_shapes.emplace_back(convertShape(material, *visual_geometry->geometry, transform));
    }

    auto& link = createActor(collision_shape, visual_shapes);

    if(urdf_link.parent_joint != nullptr)
    {
      auto transform = urdf_link.parent_joint->parent_to_joint_origin_transform;
      link.setPositionAndOrientation({static_cast<float>(transform.position.x),
                                      static_cast<float>(transform.position.y),
                                      static_cast<float>(transform.position.z)},
                                     {static_cast<float>(transform.rotation.x),
                                      static_cast<float>(transform.rotation.y),
                                      static_cast<float>(transform.rotation.z),
                                      static_cast<float>(transform.rotation.w)});
    }
    else
    {
      link.setPositionAndOrientation({static_cast<float>(0),
                                      static_cast<float>(0),
                                      static_cast<float>(0)},
                                     {static_cast<float>(0),
                                      static_cast<float>(0),
                                      static_cast<float>(0),
                                      static_cast<float>(1.)});
    }

    if(urdf_link.inertial)
    {
      const auto& inertial = *urdf_link.inertial;
      // link.setPositionAndOrientation({static_cast<float>(inertial.origin.position.x),
      //                                 static_cast<float>(inertial.origin.position.y),
      //                                 static_cast<float>(inertial.origin.position.z)},
      //                                {static_cast<float>(inertial.origin.rotation.x),
      //                                 static_cast<float>(inertial.origin.rotation.y),
      //                                 static_cast<float>(inertial.origin.rotation.z),
      //                                 static_cast<float>(inertial.origin.rotation.w)});
      link.setMass(static_cast<float>(inertial.mass));
    }

    robot.links_[urdf_link.name] = std::addressof(link);
  }

  void World::processJoints(owds::Urdf& robot, const urdf::Link& urdf_link)
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

  void World::processJoint(owds::Urdf& robot, const urdf::Joint& urdf_joint)
  {
    (void)robot;
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

  owds::Shape World::convertShape(const owds::Material& material, const urdf::Geometry& urdf_shape, glm::mat4& transform)
  {
    switch(urdf_shape.type)
    {
    case urdf::Geometry::SPHERE:
    {
      auto& sphere = dynamic_cast<const urdf::Sphere&>(urdf_shape);
      return createShapeSphere(material.diffuse_color_,
                               static_cast<float>(sphere.radius),
                               transform);
    }
    case urdf::Geometry::BOX:
    {
      auto& box = dynamic_cast<const urdf::Box&>(urdf_shape);
      return createShapeBox(material.diffuse_color_,
                            {static_cast<float>(box.dim.x),
                             static_cast<float>(box.dim.y),
                             static_cast<float>(box.dim.z)},
                            transform);
    }
    case urdf::Geometry::CYLINDER:
    {
      // I hope the cylinder is ok
      auto& cylinder = dynamic_cast<const urdf::Cylinder&>(urdf_shape);
      return createShapeCylinder(material.diffuse_color_,
                                 static_cast<float>(cylinder.radius),
                                 static_cast<float>(cylinder.length),
                                 transform);
    }
    case urdf::Geometry::MESH:
    {
      auto& mesh = dynamic_cast<const urdf::Mesh&>(urdf_shape);

      return createShapeFromModel(material,
                                  owds::rosPkgPathToPath(mesh.filename),
                                  {static_cast<float>(mesh.scale.x),
                                   static_cast<float>(mesh.scale.y),
                                   static_cast<float>(mesh.scale.z)},
                                  transform);
    }
    default:
      assert(false && "Unrecognized/unsupported shape");
    }
  }

  void World::setAmbientLight(const std::array<float, 3>& direction,
                              const std::array<float, 3>& color,
                              float ambient_strength,
                              float diffuse_strength,
                              float specular_strength)
  {
    ambient_light_ = AmbientLight(glm::vec3(direction[0], direction[1], direction[2]),
                                  glm::vec3(color[0], color[1], color[2]),
                                  ambient_strength, diffuse_strength, specular_strength);
  }

  void World::setAmbientLightDirection(const std::array<float, 3>& direction)
  {
    ambient_light_.setDirection(glm::vec3(direction[0], direction[1], direction[2]));
  }

  void World::setAmbientLightColor(const std::array<float, 3>& color)
  {
    ambient_light_.setColor(glm::vec3(color[0], color[1], color[2]));
  }

  void World::setAmbientLightAmbientStrength(float ambient_strength)
  {
    ambient_light_.setAmbientStrength(ambient_strength);
  }

  std::size_t World::addPointLight(const std::array<float, 3>& position,
                                   const std::array<float, 3>& color,
                                   float ambient_strength,
                                   float diffuse_strength,
                                   float specular_strength,
                                   float attenuation_radius)
  {
    return point_lights_.addLight(glm::vec3(position[0], position[1], position[2]),
                                  glm::vec3(color[0], color[1], color[2]),
                                  ambient_strength,
                                  diffuse_strength,
                                  specular_strength,
                                  attenuation_radius);
  }

  void World::removePointLight(std::size_t id)
  {
    point_lights_.removeLight(id);
  }

  void World::setPointLightColor(std::size_t id, const glm::vec3& color)
  {
    point_lights_.setColor(id, glm::vec3(color[0], color[1], color[2]));
  }

  void World::setPointLightPosition(std::size_t id, const glm::vec3& position)
  {
    point_lights_.setPosition(id, glm::vec3(position[0], position[1], position[2]));
  }

  void World::setPointLightAmbientStrength(std::size_t id, float strength)
  {
    point_lights_.setAmbientStrength(id, strength);
  }

  int World::addDebugText(const std::string& text,
                          const std::array<float, 3>& position,
                          float height,
                          const std::array<float, 3>& color,
                          bool centered,
                          int replace_id)
  {
    DebugText_t debug{
      text,
      centered,
      ToV3(position),
      ToV3(color),
      height};

    if(replace_id >= 0)
    {
      if(replace_id < (int)debug_texts_.size())
      {
        debug_texts_[replace_id] = debug;
        return replace_id;
      }
      else
        return -1;
    }
    else
    {
      int id = -1;
      for(size_t i = 0; i < debug_texts_.size(); i++)
      {
        if(debug_texts_[i].text.empty())
        {
          id = i;
          break;
        }
      }

      if(id >= 0)
        debug_texts_[id] = debug;
      else
      {
        id = debug_texts_.size();
        debug_texts_.emplace_back(debug);
      }
      return id;
    }
  }

  void World::removeDebugText(int id)
  {
    if(id >= 0 && id < (int)debug_texts_.size())
      debug_texts_[id].text = "";
  }

  int World::addDebugLine(const std::array<float, 3>& position_from,
                          const std::array<float, 3>& position_to,
                          const std::array<float, 3>& color,
                          int replace_id)
  {
    return addDebugLine({position_from, position_to},
                        {0, 1},
                        color, replace_id);
  }

  int World::addDebugLine(const std::vector<std::array<float, 3>>& vertices,
                          const std::vector<unsigned int>& indices,
                          const std::array<float, 3>& color,
                          int replace_id)
  {
    std::vector<glm::vec3> glm_vertices;
    glm_vertices.reserve(vertices.size());
    for(auto& vertex : vertices)
      glm_vertices.emplace_back(ToV3(vertex));

    DebugLine debug{
      glm_vertices,
      indices,
      ToV3(color)};

    if(replace_id >= 0)
    {
      if(replace_id < (int)debug_lines_.size())
      {
        debug_lines_.at(replace_id) = std::move(debug);
        return replace_id;
      }
      else
        return -1;
    }
    else
    {
      int id = -1;
      for(size_t i = 0; i < debug_lines_.size(); i++)
      {
        if(debug_lines_[i].indices_.empty())
        {
          id = i;
          break;
        }
      }

      if(id >= 0)
        debug_lines_.at(id) = std::move(debug);
      else
      {
        id = debug_lines_.size();
        debug_lines_.emplace_back(debug);
      }
      return id;
    }
  }

  void World::removeDebugLine(int id)
  {
    if(id >= 0 && id < (int)debug_lines_.size())
      debug_lines_[id].indices_.clear();
  }

} // namespace owds