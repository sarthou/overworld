#ifndef OWDS_GRAPHICS_ASSIMP_MODELLOADER_H
#define OWDS_GRAPHICS_ASSIMP_MODELLOADER_H

#include <filesystem>

#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Models/ModelLoader.h"

namespace owds::assimp {
  class ModelLoader final : public owds::ModelLoader
  {
  public:
    std::unique_ptr<owds::Model> load(const std::filesystem::path& path) const override;

  private:
    void computeTangentSpace(const std::unique_ptr<owds::Model>& model) const;
    void computeTangentSpace(Mesh& mesh) const;
  };
} // namespace owds::assimp

#endif // OWDS_GRAPHICS_ASSIMP_MODELLOADER_H
