#include "overworld/Utils/Ontology.h"

#include <array>
#include <string>

#include "overworld/Utils/RosPackage.h"
#include "overworld/Utils/Wavefront.h"

namespace owds {

  namespace ontology {

    std::array<double, 4> getEntityColor(onto::OntologyManipulator* onto, const std::string& indiv_name, const std::array<double, 4>& default_value)
    {
      auto color = onto->individuals.getOn(indiv_name, "hasColor");
      if(color.empty() == false)
      {
        auto hex_value = onto->individuals.getOn(color.front(), "hexRgbValue");
        if(hex_value.empty() == false)
        {
          unsigned int hex = 0;
          sscanf(hex_value[0].substr(hex_value[0].find('#') + 1).c_str(), "%x", &hex);
          return {((hex >> 16) & 0xff) / 255., ((hex >> 8) & 0xff) / 255., (hex & 0xff) / 255., 1.0};
        }
      }

      return default_value;
    }

    Shape_t getEntityShape(onto::OntologyManipulator* onto, const std::string& indiv_name)
    {
      auto visual_meshes = onto->individuals.getOn(indiv_name, "hasVisualMesh");
      auto collision_meshes = onto->individuals.getOn(indiv_name, "hasCollisionMesh");
      auto meshes = onto->individuals.getOn(indiv_name, "hasMesh");
      auto textures = onto->individuals.getOn(indiv_name, "hasTexture");

      Shape_t shape;
      if(meshes.empty() == false)
      {
        shape.type = SHAPE_MESH;
        if(visual_meshes.empty() == false)
          shape.visual_mesh_resource = visual_meshes.front().substr(visual_meshes.front().find('#') + 1);
        else
          shape.visual_mesh_resource = meshes.front().substr(meshes.front().find('#') + 1);
        if(collision_meshes.empty() == false)
          shape.colision_mesh_resource = collision_meshes.front().substr(collision_meshes.front().find('#') + 1);
        else
          shape.colision_mesh_resource = meshes.front().substr(meshes.front().find('#') + 1);

        shape.color = getEntityColor(onto, indiv_name);

        if(textures.empty() == false)
        {
          shape.texture = getFullPath(textures.front().substr(textures.front().find('#') + 1));
        }
      }
      else
        shape.type = SHAPE_NONE;

      return shape;
    }

    double getEntityMass(onto::OntologyManipulator* onto, const std::string& indiv_name)
    {
      auto masses = onto->individuals.getOn(indiv_name, "hasMass");

      if(masses.empty() == false)
      {
        auto mass_str = masses.front().substr(masses.front().find('#') + 1);
        return std::stod(mass_str);
      }
      else
        return 0;
    }

    void addColor(onto::OntologyManipulator* onto, const std::string& color_name, const std::string& rgb_value)
    {
      if(onto->individuals.exist(color_name) == false)
      {
        onto->feeder.addInheritage(color_name, "Color");
        if(rgb_value.empty() == false)
          onto->feeder.addRelation(color_name, "hexRgbValue", "hexrbg", rgb_value);
        onto->feeder.waitUpdate(1000);
      }
    }

    void addColorToEntity(onto::OntologyManipulator* onto, const std::string& indiv_name, const std::string& color_name)
    {
      onto->feeder.addRelation(indiv_name, "hasColor", color_name);
      onto->feeder.waitUpdate(1000);
    }

  } // namespace ontology

} // namespace owds