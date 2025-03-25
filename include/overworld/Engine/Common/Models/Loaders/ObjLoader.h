#ifndef OVERWORLD_OBJLOADER_H
#define OVERWORLD_OBJLOADER_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "overworld/Engine/Common/Models/Loaders/TinyObjReader.h"
#include "overworld/Engine/Common/Models/Model.h"

namespace owds {

  class ObjLoader : public tinyobj::ObjReader
  {
  public:
    std::unique_ptr<owds::Model> read(const std::string& path);

  private:
    Vertex getVertex(const tinyobj::Attrib_t& attribute,
                     const tinyobj::Index_t& indexes);

    std::unique_ptr<owds::Model> getModel(const tinyobj::Attrib_t& attribute,
                                          std::vector<tinyobj::Mesh_t>& meshes,
                                          std::vector<tinyobj::Material_t>& materials);
  };

} // namespace owds

#endif // OVERWORLD_OBJLOADER_H