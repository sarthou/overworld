#include "overworld/Engine/Common/Urdf/UrdfLoader.h"

#include <fstream>
#include <glm/vec4.hpp>
#include <map>
#include <string>
#include <tinyxml.h>
#include <vector>

#include "overworld/Engine/Common/Models/Material.h"
#include "overworld/Utils/ROS.h"
#include "overworld/Utils/ShellDisplay.h"
#include "overworld/Utils/XmlTokenize.h"

namespace owds {

  urdf::Urdf_t UrdfLoader::read(const std::string& path)
  {
    urdf::Urdf_t urdf;

    TiXmlDocument doc;
    if(getXmlDocument(path, doc) == false)
      return urdf;

    urdf.name = doc.RootElement()->Attribute("name");

    auto material_library = getMaterialLibrary(doc.RootElement());
    urdf.links = getLinks(doc.RootElement(), material_library);
    urdf.joints = getJoints(doc.RootElement());

    return urdf;
  }

  bool UrdfLoader::getXmlDocument(const std::string& path, TiXmlDocument& doc)
  {
    std::string content;
    std::ifstream f(path);

    if(!f.is_open())
    {
      ShellDisplay::error("Fail to open : " + path);
      return false;
    }

    std::string tmp;
    while(std::getline(f, tmp))
      content += tmp;

    // removeDocType(response);

    doc.Parse((const char*)content.c_str(), nullptr, TIXML_ENCODING_UTF8);

    TiXmlElement* root = doc.RootElement();
    if(root == nullptr)
      return false;

    return true;
  }

  std::map<std::string, Material> UrdfLoader::getMaterialLibrary(TiXmlElement* root)
  {
    std::map<std::string, Material> res;

    for(auto* material = root->FirstChildElement("material"); material != nullptr; material = material->NextSiblingElement("material"))
      res.emplace(getMaterial(material));

    return res;
  }

  std::pair<std::string, Material> UrdfLoader::getMaterial(TiXmlElement* element)
  {
    Material instance;
    std::string id = std::string(element->Attribute("name"));

    auto* color = element->FirstChildElement("color");
    if(color != nullptr)
    {
      std::string rgba = std::string(color->Attribute("rgba"));
      auto color_vec = getVector4FromXmlText(rgba.c_str());
      instance.diffuse_color_.r_ = color_vec.x;
      instance.diffuse_color_.g_ = color_vec.y;
      instance.diffuse_color_.b_ = color_vec.z;
      instance.diffuse_color_.a_ = color_vec.w;
    }

    auto* texture = element->FirstChildElement("texture");
    if(texture != nullptr)
    {
      std::string filename = std::string(texture->Attribute("filename"));
      instance.diffuse_texture_ = rosPkgPathToPath(filename);
    }

    return {id, instance};
  }

  std::map<std::string, urdf::Link_t> UrdfLoader::getLinks(TiXmlElement* root, const std::map<std::string, Material>& materials)
  {
    std::map<std::string, urdf::Link_t> res;

    for(auto* link_elem = root->FirstChildElement("link"); link_elem != nullptr; link_elem = link_elem->NextSiblingElement("link"))
    {
      urdf::Link_t link;
      link.name = std::string(link_elem->Attribute("name"));

      auto* inertia_elem = link_elem->FirstChildElement("inertial");
      if(inertia_elem != nullptr)
        link.inertia = getInertia(inertia_elem);

      auto* visual_elem = link_elem->FirstChildElement("visual");
      if(visual_elem != nullptr)
        link.visual = getGeometry(visual_elem, materials);

      for(auto* collision_elem = link_elem->FirstChildElement("collision"); collision_elem != nullptr; collision_elem = collision_elem->NextSiblingElement("collision"))
        link.collisions.push_back(getGeometry(collision_elem, materials));

      res.emplace(link.name, link);
    }

    return res;
  }

  urdf::Inertia_t UrdfLoader::getInertia(TiXmlElement* element)
  {
    urdf::Inertia_t res;

    auto* mass = element->FirstChildElement("mass");
    if(mass != nullptr)
    {
      const char* value_text = mass->Attribute("value");
      res.mass = std::atof(value_text);
    }

    auto* origin = element->FirstChildElement("origin");
    if(origin != nullptr)
    {
      res.origin = getVector3FromXmlText(origin->Attribute("xyz"));
      // should we read rpy ?
    }

    auto* inertia = element->FirstChildElement("inertia");
    if(inertia != nullptr)
    {
      const char* value_text = inertia->Attribute("ixx");
      res.ix.x = std::atof(value_text);
      value_text = inertia->Attribute("ixy");
      res.ix.y = std::atof(value_text);
      value_text = inertia->Attribute("ixz");
      res.ix.z = std::atof(value_text);

      value_text = inertia->Attribute("iyy");
      res.iy.y = std::atof(value_text);
      value_text = inertia->Attribute("iyz");
      res.iy.z = std::atof(value_text);

      value_text = inertia->Attribute("izz");
      res.iz.z = std::atof(value_text);
    }

    return res;
  }

  urdf::Geometry_t UrdfLoader::getGeometry(TiXmlElement* element, const std::map<std::string, Material>& materials)
  {
    urdf::Geometry_t res;

    auto* origin = element->FirstChildElement("origin");
    if(origin != nullptr)
    {
      auto* translation = origin->Attribute("xyz");
      if(translation != nullptr)
        res.origin_translation = getVector3FromXmlText(translation);
      auto* rotation = origin->Attribute("rpy");
      if(rotation != nullptr)
        res.origin_rotation = getVector3FromXmlText(rotation);
    }

    auto* material_elem = element->FirstChildElement("material");
    if(material_elem != nullptr)
    {
      auto material = getMaterial(material_elem);
      auto material_it = materials.find(material.first);
      if(material_it != materials.end())
        res.material = material_it->second;
      else
        res.material = material.second;
    }

    auto* geometry_elem = element->FirstChildElement("geometry");
    if(geometry_elem != nullptr)
    {
      auto* geometry_type = geometry_elem->FirstChildElement();
      std::string type = geometry_type->ValueStr();
      if(type == "mesh")
      {
        res.type = urdf::GeometryType_e::geometry_mesh;
        res.file_name = geometry_type->Attribute("filename");
        auto* scale = geometry_type->Attribute("scale");
        if(scale != nullptr)
          res.scale = getVector3FromXmlText(scale);
      }
      else if(type == "box")
      {
        res.type = urdf::GeometryType_e::geometry_box;
        res.scale = getVector3FromXmlText(geometry_type->Attribute("size"));
      }
      else if(type == "sphere")
      {
        res.type = urdf::GeometryType_e::geometry_sphere;
        const char* radius_text = geometry_type->Attribute("radius");
        res.scale.x = std::atof(radius_text);
        res.scale.y = 0.;
        res.scale.z = 0.;
      }
      else if(type == "cylinder")
      {
        res.type = urdf::GeometryType_e::geometry_cylinder;
        const char* radius_text = geometry_type->Attribute("radius");
        res.scale.x = std::atof(radius_text);
        const char* length_text = geometry_type->Attribute("length");
        res.scale.y = std::atof(length_text);
        res.scale.z = 0.;
      }
    }

    return res;
  }

  std::map<std::string, urdf::Joint_t> UrdfLoader::getJoints(TiXmlElement* root)
  {
    std::map<std::string, urdf::Joint_t> res;

    for(auto* joint_elem = root->FirstChildElement("joint"); joint_elem != nullptr; joint_elem = joint_elem->NextSiblingElement("joint"))
    {
      urdf::Joint_t joint;
      joint.name = std::string(joint_elem->Attribute("name"));
      std::string type = std::string(joint_elem->Attribute("type"));

      if(type == "revolute")
        joint.type = urdf::JointType_e::joint_revolute;
      else if(type == "continuous")
        joint.type = urdf::JointType_e::joint_continuous;
      else if(type == "prismatic")
        joint.type = urdf::JointType_e::joint_prismatic;
      else if(type == "fixed")
        joint.type = urdf::JointType_e::joint_fixed;
      else if(type == "floating")
        joint.type = urdf::JointType_e::joint_floating;
      else if(type == "planar")
        joint.type = urdf::JointType_e::joint_planar;

      joint.parent_link = joint_elem->FirstChildElement("parent")->Attribute("link");
      joint.child_link = joint_elem->FirstChildElement("child")->Attribute("link");

      auto* origin = joint_elem->FirstChildElement("origin");
      if(origin != nullptr)
      {
        auto* translation = origin->Attribute("xyz");
        if(translation != nullptr)
          joint.origin_translation = getVector3FromXmlText(translation);
        auto* rotation = origin->Attribute("rpy");
        if(rotation != nullptr)
          joint.origin_rotation = getVector3FromXmlText(rotation);
      }

      auto* axis = joint_elem->FirstChildElement("axis");
      if(axis != nullptr)
      {
        joint.axis = getVector3FromXmlText(axis->Attribute("xyz"));
      }

      res.emplace(joint.name, joint);
    }

    return res;
  }

} // namespace owds