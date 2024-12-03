#ifndef OWDS_GRAPHICS_OPENGL_RENDERER_H
#define OWDS_GRAPHICS_OPENGL_RENDERER_H

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "overworld/Engine/Common/Camera/Camera.h"
#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Shapes/Shape.h"
#include "overworld/Engine/Common/Urdf/Actor.h"
#include "overworld/Engine/Graphics/Common/InstanceData.h"
#include "overworld/Engine/Graphics/OpenGL/AmbientShadow.h"
#include "overworld/Engine/Graphics/OpenGL/Cubemap.h"
#include "overworld/Engine/Graphics/OpenGL/LinesHandle.h"
#include "overworld/Engine/Graphics/OpenGL/MeshHandle.h"
#include "overworld/Engine/Graphics/OpenGL/OffScreen.h"
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
    static void init();
    static void release();

    bool initialize(Window& window);
    void attachWorld(World* world) { world_ = world; }

    void commit();

    void setRenderCamera(Camera* camera);

  private:
    World* world_ = nullptr;
    Camera render_camera_;
    std::unordered_map<std::string, Shader> shaders_;
    Screen screen_;
    std::vector<OffScreen> off_screens_;
    Cubemap sky_;
    AmbientShadow shadow_;
    PointShadow point_shadows_;
    TextRenderer text_renderer_;
    float max_fps_ = 120;
    float delta_time_;
    float last_frame_;

    bool render_collision_models_ = false;

    std::unordered_map<std::string, Texture2D> loaded_textures_;
    std::unordered_map<Model::Id, std::unordered_map<Mesh::Id, MeshHandle>> cached_models_;
    std::unordered_map<Model::Id, std::unordered_map<Mesh::Id, std::vector<InstanceData>>> current_mesh_batches_;
    std::unordered_map<unsigned int, LinesHandle> cached_lines_;
    std::unordered_map<unsigned int, LinesHandle> debug_axis_;

    void loadWorld();
    void loadActor(Actor* actor, const ShapeBox& shape);
    void loadActor(Actor* actor, const ShapeCapsule& shape);
    void loadActor(Actor* actor, const ShapeCustomMesh& shape);
    void loadActor(Actor* actor, const ShapeCylinder& shape);
    void loadActor(Actor* actor, const ShapeDummy& shape);
    void loadActor(Actor* actor, const ShapeSphere& shape);
    void loadInstance(const Model& model, const Material& material, const glm::mat4& model_mat);
    Material combineMaterials(const Material& shape_material, const Material& model_material);
    std::vector<Texture2D> loadTextures(Material& material);
    void loadModel(const Model& model, const Material& material);
    void loadDebugLines();
    void render();

    void renderModels(const Shader& shader, unsigned int texture_offset = 0);
    void renderModelsSegmented(const Shader& shader);

    void renderMainScreen(bool render_shadows);
    void renderOffScreens(bool render_shadows);
    void renderOffscreenRgb(Camera* camera, bool render_shadows);
    void renderOffscreenSegmented(Camera* camera);
    void renderShadowDepth();
    void renderDebug();

    void setLightsUniforms(const Shader& shader, bool use_ambient_shadows = true, bool use_points_shadows = true);
    void setAntiAliasing(ViewAntiAliasing_e setting);
  };
} // namespace owds

#endif // OWDS_GRAPHICS_OPENGL_RENDERER_H
