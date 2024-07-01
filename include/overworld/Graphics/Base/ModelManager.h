#ifndef OWDS_GRAPHICS_BASE_MODELMANAGER_H
#define OWDS_GRAPHICS_BASE_MODELMANAGER_H

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>

namespace owds {
  class ModelLoader;
  class Model;

  class ModelManager
  {
    ModelManager();

  public:
    ~ModelManager() noexcept;
    ModelManager(const ModelManager& other) = delete;
    ModelManager(ModelManager&& other) = delete;

    static ModelManager& get();

    void setModelLoader(std::unique_ptr<ModelLoader> loader);

    template<typename T>
    void setModelLoader() { setModelLoader(std::make_unique<T>()); }

    owds::Model& load(const std::filesystem::path& path);

  protected:
    std::unique_ptr<ModelLoader> model_loader_;
    std::unordered_map<std::string, std::unique_ptr<owds::Model>> models_;
  };
} // namespace owds

#endif // OWDS_GRAPHICS_BASE_MODELMANAGER_H
