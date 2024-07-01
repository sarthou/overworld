#ifndef OWDS_GRAPHICS_ASSIMP_MODELLOADER_H
#define OWDS_GRAPHICS_ASSIMP_MODELLOADER_H

#include "overworld/Graphics/Base/ModelLoader.h"

namespace owds::assimp {
  class ModelLoader final : public owds::ModelLoader
  {
  public:
    [[nodiscard]] std::unique_ptr<owds::Model> load(const std::filesystem::path& path) const override;
  };
} // namespace owds::assimp

#endif // OWDS_GRAPHICS_ASSIMP_MODELLOADER_H
