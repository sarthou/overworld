#include "overworld/Engine/Common/World.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <string>

#include "overworld/Compat/ROS.h"
#include "overworld/Engine/Common/Models/Color.h"
#include "overworld/Engine/Common/Models/Material.h"
#include "overworld/Engine/Common/Models/ModelManager.h"
#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Common/Urdf/Urdf.h"
#include "overworld/Engine/Common/Urdf/UrdfLoader.h"
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
    UrdfLoader loader;
    urdf::Urdf_t urdf_model;

    if(from_base_path)
      urdf_model = loader.read((base_assets_path_ / path).string());
    else
      urdf_model = loader.read(path);

    auto urdf = std::make_unique<owds::Urdf>();
    const auto urdf_ptr = urdf.get();
    loaded_urdfs_[urdf_ptr] = std::move(urdf);

    urdf_ptr->name_ = urdf_model.name;

    for(auto& [name, link] : urdf_model.links)
    {
      processLink(*urdf_ptr, link);
    }

    for(const auto& [name, joint] : urdf_model.joints)
    {
      processJoint(*urdf_ptr, joint);
    }

    return *urdf_ptr;
  }

  void World::processLink(owds::Urdf& robot, const urdf::Link_t& urdf_link)
  {
    owds::Shape collision_shape = owds::ShapeDummy();
    std::vector<owds::Shape> visual_shapes;

    if(urdf_link.collisions.size())
    {
      auto& collision = urdf_link.collisions.front();
      if(collision.type != urdf::GeometryType_e::geometry_none)
      {
        glm::mat4 translate = glm::translate(glm::mat4(1), collision.origin_translation);
        glm::mat4 rotate = glm::mat4_cast(glm::quat(collision.origin_rotation));
        glm::mat4 transform = translate * rotate;
        (void)transform;
        // collision_shape = convertShape(collision, transform); // TODO
      }
    }

    assert(urdf_link.collisions.size() <= 1 && "Links with multiple collision shapes are not supported at this moment.");

    {
      if(urdf_link.visual.type != urdf::GeometryType_e::geometry_none)
      {
        auto& visual = urdf_link.visual;
        glm::mat4 translate = glm::translate(glm::mat4(1), visual.origin_translation);
        glm::mat4 rotate = glm::mat4_cast(glm::quat(visual.origin_rotation));
        glm::mat4 transform = translate * rotate;
        visual_shapes.emplace_back(convertShape(visual, transform));
      }
    }

    auto& link = createActor(collision_shape, visual_shapes);

    link.setPositionAndOrientation({static_cast<float>(0),
                                    static_cast<float>(0),
                                    static_cast<float>(0)},
                                   {static_cast<float>(0),
                                    static_cast<float>(0),
                                    static_cast<float>(0),
                                    static_cast<float>(1.)}); // TODO where to do it well ?

    link.setMass(static_cast<float>(urdf_link.inertia.mass));

    robot.links_[urdf_link.name] = std::addressof(link);
  }

  void World::processJoint(owds::Urdf& robot, const urdf::Joint_t& urdf_joint)
  {
    (void)robot;
    // auto& link0 = *robot.links_.at(urdf_joint.parent_link_name);
    // auto& link1 = *robot.links_.at(urdf_joint.child_link_name);

    // joint.parent_to_joint_origin_transform;
    switch(urdf_joint.type)
    {
    case urdf::JointType_e::joint_revolute:
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
    case urdf::JointType_e::joint_continuous:
    {
      break;
    }
    case urdf::JointType_e::joint_prismatic:
    {
      break;
    }
    case urdf::JointType_e::joint_floating:
    {
      break;
    }
    case urdf::JointType_e::joint_planar:
    {
      break;
    }
    case urdf::JointType_e::joint_fixed:
    {
      break;
    }
    case urdf::JointType_e::joint_none:
    default:
    {
      assert(false && "Unrecognized/unsupported joint type.");
    }
    }
  }

  owds::Shape World::convertShape(const urdf::Geometry_t& urdf_shape, glm::mat4& transform)
  {
    switch(urdf_shape.type)
    {
    case urdf::GeometryType_e::geometry_sphere:
    {
      return createShapeSphere(urdf_shape.material.diffuse_color_,
                               static_cast<float>(urdf_shape.scale.x),
                               transform);
    }
    case urdf::GeometryType_e::geometry_box:
    {
      return createShapeBox(urdf_shape.material.diffuse_color_,
                            {static_cast<float>(urdf_shape.scale.x),
                             static_cast<float>(urdf_shape.scale.y),
                             static_cast<float>(urdf_shape.scale.z)},
                            transform);
    }
    case urdf::GeometryType_e::geometry_cylinder:
    {
      return createShapeCylinder(urdf_shape.material.diffuse_color_,
                                 static_cast<float>(urdf_shape.scale.x),
                                 static_cast<float>(urdf_shape.scale.y),
                                 transform);
    }
    case urdf::GeometryType_e::geometry_mesh:
    {
      return createShapeFromModel(urdf_shape.material,
                                  owds::rosPkgPathToPath(urdf_shape.file_name),
                                  {static_cast<float>(urdf_shape.scale.x),
                                   static_cast<float>(urdf_shape.scale.y),
                                   static_cast<float>(urdf_shape.scale.z)},
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