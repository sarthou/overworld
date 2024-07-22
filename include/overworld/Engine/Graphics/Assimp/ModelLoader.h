#ifndef OWDS_GRAPHICS_ASSIMP_MODELLOADER_H
#define OWDS_GRAPHICS_ASSIMP_MODELLOADER_H

#include <filesystem>

#include "overworld/Engine/Common/Models/ModelLoader.h"

namespace owds::assimp {
  class ModelLoader final : public owds::ModelLoader
  {
  public:
    std::unique_ptr<owds::Model> load(const std::filesystem::path& path) const override;
  };
} // namespace owds::assimp

#endif // OWDS_GRAPHICS_ASSIMP_MODELLOADER_H
