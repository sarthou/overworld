#include "overworld/Graphics/Base/ModelManager.h"

#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Graphics/Base/ModelLoader.h"

namespace owds {
  ModelManager::ModelManager() = default;
  ModelManager::~ModelManager() noexcept = default;

  ModelManager& ModelManager::get()
  {
    static ModelManager mgr{};
    return mgr;
  }

  void ModelManager::setModelLoader(std::unique_ptr<ModelLoader> loader)
  {
    model_loader_ = std::move(loader);
  }

  owds::Model& ModelManager::load(const std::filesystem::path& path)
  {
    const auto absolute_path_str = path.string();

    if(!models_.count(absolute_path_str))
    {
      assert(model_loader_ && "You must register a model loader via owds::ModelManager::load(...) first!");

      auto model = model_loader_->load(path);

      assert(model && "Failed to load model");

      models_[absolute_path_str] = std::move(model);
    }

    return *models_[absolute_path_str];
  }
} // namespace owds