
#ifndef OWDS_GRAPHICS_OPENGL_SHADER_H
#define OWDS_GRAPHICS_OPENGL_SHADER_H

#include <glm/glm.hpp>
#include <string>

#include "glad/glad.h" // include glad to get all the required OpenGL headers

namespace owds {

  class Shader
  {
  public:
    static std::string shaders_directory;

    // the program ID
    unsigned int id_;

    // constructor reads and builds the shader
    Shader(const std::string& vertex_path, const std::string& fragment_path, const std::string& geometry_path = "");
    // use/activate the shader
    void use();
    // utility uniform functions
    void setBool(const std::string& name, bool value) const;
    void setInt(const std::string& name, int value) const;
    void setFloat(const std::string& name, float value) const;
    void setMat4(const std::string& name, const glm::mat4& value) const;
    void setVec3(const std::string& name, const glm::vec3& value) const;
    void setVec4(const std::string& name, const glm::vec4& value) const;

  private:
    void checkCompileErrors(GLuint shader, std::string type);
  };

} // namespace owds

#endif // OWDS_GRAPHICS_OPENGL_SHADER_H