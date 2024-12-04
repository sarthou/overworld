#ifndef OWDS_URDFLOQDER_H
#define OWDS_URDFLOQDER_H

#include <glm/vec3.hpp>
#include <map>
#include <string>
#include <tinyxml.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "overworld/Engine/Common/Models/Material.h"

namespace owds {

  namespace urdf {

    enum GeometryType_e
    {
      geometry_none,
      geometry_mesh,
      geometry_box,
      geometry_cylinder,
      geometry_sphere
    };

    enum JointType_e
    {
      joint_none,
      joint_revolute,
      joint_continuous,
      joint_prismatic,
      joint_fixed,
      joint_floating,
      joint_planar
    };

    struct Inertia_t
    {
      float mass = 0.;
      glm::vec3 origin;
      glm::vec3 ix;
      glm::vec3 iy;
      glm::vec3 iz;
    };

    struct Geometry_t
    {
      Material material;
      GeometryType_e type = geometry_none;
      std::string file_name;
      glm::vec3 origin_translation;
      glm::vec3 origin_rotation;
      glm::vec3 scale = {1., 1., 1.}; // x = radius y = length for cylinder and sphere

      Geometry_t() = default;

      Geometry_t(const std::string& file,
                 glm::vec3 origin_translation = {0., 0., 0.},
                 glm::vec3 origin_rotation = {0., 0., 0.},
                 glm::vec3 scale = {1., 1., 1.}) : type(geometry_mesh),
                                                   file_name(file),
                                                   origin_translation(origin_translation),
                                                   origin_rotation(origin_rotation),
                                                   scale(scale)
      {}
    };

    struct Link_t
    {
      std::string name;
      Inertia_t inertia;
      Geometry_t visual;
      std::vector<Geometry_t> collisions;
    };

    struct Joint_t
    {
      std::string name;
      JointType_e type = joint_none;
      glm::vec3 origin_translation;
      glm::vec3 origin_rotation;
      std::string parent_link;
      std::string child_link;
      glm::vec3 axis = {1., 0., 0.};
    };

    struct Urdf_t
    {
      std::string name;
      std::string root_link;
      std::map<std::string, Joint_t> joints;
      std::map<std::string, Link_t> links;
      std::unordered_map<std::string, std::unordered_set<std::string>> tree; // link to joints
      std::unordered_map<std::string, std::string> link_to_parent_joint;
    };

  } // namespace urdf

  class UrdfLoader
  {
  public:
    urdf::Urdf_t read(const std::string& path);

  private:
    bool getXmlDocument(const std::string& path, TiXmlDocument& doc);
    std::map<std::string, Material> getMaterialLibrary(TiXmlElement* root);
    std::pair<std::string, Material> getMaterial(TiXmlElement* element);
    std::map<std::string, urdf::Link_t> getLinks(TiXmlElement* root, const std::map<std::string, Material>& materials);
    urdf::Inertia_t getInertia(TiXmlElement* element);
    urdf::Geometry_t getGeometry(TiXmlElement* element, const std::map<std::string, Material>& materials);
    std::map<std::string, urdf::Joint_t> getJoints(TiXmlElement* root);
    std::unordered_set<std::string> findRootLink(const std::map<std::string, urdf::Joint_t>& joints);
    void getTree(urdf::Urdf_t& model);
  };

} // namespace owds

#endif // OWDS_URDFLOQDER_H