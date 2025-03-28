#ifndef OWDS_GRAPHICS_OPENGL_MESHHANDLE_H
#define OWDS_GRAPHICS_OPENGL_MESHHANDLE_H

#include <unordered_map>
#include <vector>

#include "overworld/Engine/Common/Models/Color.h"
#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Graphics/OpenGL/Shader.h"

namespace owds {

  class Texture2D;

  struct MeshMaterialHandle_t
  {
    std::vector<Texture2D> textures;
    float shininess;
    float specular;
    Color color;
  };

  class MeshHandle
  {
  public:
    uint32_t id;
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::unordered_map<uint32_t, MeshMaterialHandle_t> materials;

    MeshHandle(const Mesh& mesh);

    void drawId(const Shader& shader, uint32_t model_id) const;

    void draw(const Shader& shader, uint32_t model_id, unsigned int texture_pose_offset = 0) const;

  private:
    unsigned int vbo_; // vertex buffer object
    unsigned int ebo_; //
    unsigned int vao_; // vertex array object

    // initializes all the buffer objects/arrays
    void setupMesh();
  };

} // namespace owds

#endif // OWDS_GRAPHICS_OPENGL_MESHHANDLE_H
