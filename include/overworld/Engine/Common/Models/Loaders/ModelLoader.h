#ifndef OWDS_COMMON_MODELLOADER_H
#define OWDS_COMMON_MODELLOADER_H

#include <filesystem>
#include <memory>

#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Common/Models/Model.h"

namespace owds {

  class ModelLoader
  {
  public:
    std::unique_ptr<owds::Model> load(const std::filesystem::path& path) const;

  private:
    void computeTangentSpace(const std::unique_ptr<owds::Model>& model) const;
    void computeTangentSpace(Mesh& mesh) const;
  };

} // namespace owds

#endif // OWDS_COMMON_MODELLOADER_H
