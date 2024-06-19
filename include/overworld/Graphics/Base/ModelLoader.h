#ifndef OWDS_GRAPHICS_BASE_MODELLOADER_H
#define OWDS_GRAPHICS_BASE_MODELLOADER_H

#include <filesystem>
#include <memory>

namespace owds {
  class Model;

  class ModelLoader
  {
  public:
    virtual ~ModelLoader() = default;

    [[nodiscard]] virtual std::unique_ptr<owds::Model> load(const std::filesystem::path& path) const = 0;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_MODELLOADER_H
