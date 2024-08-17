#include "overworld/Engine/Common/Models/ModelManager.h"

#include "overworld/Engine/Common/Models/Loaders/ModelLoader.h"
#include "overworld/Engine/Common/Models/Model.h"

namespace owds {
  ModelManager::ModelManager() = default;
  ModelManager::~ModelManager() noexcept = default;

  ModelManager& ModelManager::get()
  {
    static ModelManager mgr{};
    return mgr;
  }

  owds::Model& ModelManager::load(const std::filesystem::path& path)
  {
    const auto absolute_path_str = path.string();

    if(!models_.count(absolute_path_str))
    {
      auto model = model_loader_.load(path);

      assert(model && "Failed to load model");

      models_[absolute_path_str] = std::move(model);
    }

    return *models_[absolute_path_str];
  }
} // namespace owds