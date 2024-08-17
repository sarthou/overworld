#ifndef OWDS_COMMON_MODELMANAGER_H
#define OWDS_COMMON_MODELMANAGER_H

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>

#include "overworld/Engine/Common/Models/Loaders/ModelLoader.h"

namespace owds {
  class Model;

  class ModelManager
  {
    ModelManager();

  public:
    ~ModelManager() noexcept;
    ModelManager(const ModelManager& other) = delete;
    ModelManager(ModelManager&& other) = delete;

    static ModelManager& get();

    owds::Model& load(const std::filesystem::path& path);

  protected:
    ModelLoader model_loader_;
    std::unordered_map<std::string, std::unique_ptr<owds::Model>> models_;
  };
} // namespace owds

#endif // OWDS_COMMON_MODELMANAGER_H
