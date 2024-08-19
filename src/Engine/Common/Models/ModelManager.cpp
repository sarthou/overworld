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

    mut_.lock();
    if(!models_.count(absolute_path_str))
    {
      auto model = model_loader_.load(path);

      assert(model && "Failed to load model");

      models_[absolute_path_str] = std::move(model);
    }

    owds::Model& res = *models_[absolute_path_str];
    mut_.unlock();

    return res;
  }
} // namespace owds