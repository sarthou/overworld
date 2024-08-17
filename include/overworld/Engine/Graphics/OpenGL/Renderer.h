#ifndef OWDS_GRAPHICS_OPENGL_RENDERER_H
#define OWDS_GRAPHICS_OPENGL_RENDERER_H

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Graphics/Common/InstanceData.h"
#include "overworld/Engine/Graphics/OpenGL/AmbientShadow.h"
#include "overworld/Engine/Graphics/OpenGL/Camera.h"
#include "overworld/Engine/Graphics/OpenGL/Cubemap.h"
#include "overworld/Engine/Graphics/OpenGL/LinesHandle.h"
#include "overworld/Engine/Graphics/OpenGL/MeshHandle.h"
#include "overworld/Engine/Graphics/OpenGL/PointShadow.h"
#include "overworld/Engine/Graphics/OpenGL/Screen.h"
#include "overworld/Engine/Graphics/OpenGL/Shader.h"
#include "overworld/Engine/Graphics/OpenGL/TextRenderer.h"

namespace owds {
  class Window;
  class World;
  class AmbientLight;
  class PointLights;

  class Renderer
  {
  public:
    ~Renderer();
    bool initialize(const Window& window);
    void attachWorld(World* world) { world_ = world; }
    void cleanup();
    void notifyResize(std::uint32_t new_width, std::uint32_t new_height);

    void commit();

    Camera* getRenderCamera() { return &render_camera_; }

  private:
    World* world_ = nullptr;
    Camera render_camera_;
    std::unordered_map<std::string, Shader> shaders_;
    Screen screen_;
    Cubemap sky_;
    AmbientShadow shadow_;
    PointShadow point_shadows_;
    TextRenderer text_renderer_;
    float max_fps_ = 120;
    float delta_time_;
    float last_frame_;

    bool render_debug_ = true;
    bool render_collision_models_ = false;
    std::unordered_map<std::string, Texture2D> loaded_textures_;
    std::unordered_map<Model::Id, std::unordered_map<Mesh::Id, MeshHandle>> cached_models_;
    std::unordered_map<Model::Id, std::unordered_map<Mesh::Id, std::vector<InstanceData>>> current_mesh_batches_;
    std::unordered_map<unsigned int, LinesHandle> cached_lines_;

    void loadWorld();
    void loadActor(const Actor& actor, const ShapeBox& shape);
    void loadActor(const Actor& actor, const ShapeCapsule& shape);
    void loadActor(const Actor& actor, const ShapeCustomMesh& shape);
    void loadActor(const Actor& actor, const ShapeCylinder& shape);
    void loadActor(const Actor& actor, const ShapeDummy& shape);
    void loadActor(const Actor& actor, const ShapeSphere& shape);
    void loadInstance(const Model& model, const Material& material, const glm::mat4& model_mat);
    Material combineMaterials(const Material& shape_material, const Material& model_material);
    std::vector<Texture2D> loadTextures(Material& material);
    void loadModel(const Model& model, const Material& material);
    void loadDebugLines();
    void render();
    void renderModels(const Shader& shader, unsigned int texture_offset = 0);

    void setLightsUniforms(const Shader& shader);
    void setAntiAliasing(ViewAntiAliasing_e setting);
  };
} // namespace owds

#endif // OWDS_GRAPHICS_OPENGL_RENDERER_H
