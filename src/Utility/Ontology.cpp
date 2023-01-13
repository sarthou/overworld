#include "overworld/Utility/Ontology.h"

#include "overworld/Utility/Wavefront.h"
#include "overworld/Utility/RosFiles.h"

namespace owds {

namespace ontology {

std::array<double, 3> getEntityColor(OntologyManipulator* onto, const std::string& indiv_name, const std::array<double, 3>& default_value)
{
  auto color = onto->individuals.getOn(indiv_name, "hasColor");
  if(color.size() != 0)
  {
    auto hex_value = onto->individuals.getOn(color.front(), "hexRgbValue");
    if(hex_value.size())
    {
      unsigned int hex = 0;
      sscanf(hex_value[0].substr(hex_value[0].find("#") + 1).c_str(), "%x", &hex);
      return { ((hex >> 16) & 0xff) / 255., ((hex >> 8) & 0xff) / 255., (hex & 0xff) / 255. };
    }
  }
  
  return default_value;
}

Shape_t getEntityShape(OntologyManipulator* onto, const std::string& indiv_name)
{
  auto visual_meshes = onto->individuals.getOn(indiv_name, "hasVisualMesh");
  auto collision_meshes = onto->individuals.getOn(indiv_name, "hasCollisionMesh");
  auto meshes = onto->individuals.getOn(indiv_name, "hasMesh");
  auto textures = onto->individuals.getOn(indiv_name, "hasTexture");

  Shape_t shape;
  if(meshes.size())
  {
    shape.type = SHAPE_MESH;
    if(visual_meshes.size())
        shape.visual_mesh_resource = visual_meshes.front().substr(visual_meshes.front().find("#") + 1);
    else
        shape.visual_mesh_resource = meshes.front().substr(meshes.front().find("#") + 1);
    if(collision_meshes.size())
        shape.colision_mesh_resource = collision_meshes.front().substr(collision_meshes.front().find("#") + 1);
    else
        shape.colision_mesh_resource = meshes.front().substr(meshes.front().find("#") + 1);

    bool is_wavefront = wavefront::isWavefront(shape.visual_mesh_resource);
    if(is_wavefront)
    {
      auto full_path = getFullPath(shape.visual_mesh_resource);
      auto mlt = wavefront::getMltFile(full_path);
      if(mlt != "")
      {
        auto materials = wavefront::getMltMaterials(mlt);
        if(materials.size() == 1)
        {
          if(wavefront::getMaterialTexture(mlt, materials.front()) != "")
            shape.color = {1, 1, 1};
          else
            shape.color = getEntityColor(onto, indiv_name);
        }
        else
          shape.color = getEntityColor(onto, indiv_name);
      }
      else
        shape.color = getEntityColor(onto, indiv_name);
    }
    else
      shape.color = getEntityColor(onto, indiv_name);

    if(is_wavefront && textures.size())
        shape.texture = textures.front().substr(textures.front().find("#") + 1);
  }
  else
    shape.type = SHAPE_NONE;

  return shape;
}

double getEntityMass(OntologyManipulator* onto, const std::string& indiv_name)
{
  auto masses = onto->individuals.getOn(indiv_name, "hasMass");

  if(masses.size())
  {
    auto mass_str = masses.front().substr(masses.front().find("#") + 1);
    return std::stod(mass_str);
  }
  else
    return 0;
}

void addColor(OntologyManipulator* onto, const std::string& color_name, const std::string& rgb_value)
{
  if(onto->individuals.exist(color_name) == false)
  {
    onto->feeder.addInheritage(color_name, "Color");
    if(rgb_value != "")
      onto->feeder.addProperty(color_name, "hexRgbValue", "hexrbg", rgb_value);
    onto->feeder.waitUpdate(1000);
  }
}

void addColorToEntity(OntologyManipulator* onto, const std::string& indiv_name, const std::string& color_name)
{
  onto->feeder.addProperty(indiv_name, "hasColor", color_name);
  onto->feeder.waitUpdate(1000);
}

} // namespace ontology

} // namespace owds