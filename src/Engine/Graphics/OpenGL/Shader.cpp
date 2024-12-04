#include "overworld/Engine/Graphics/OpenGL/Shader.h"

#include <fstream>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float2.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/ext/vector_float4.hpp>
#include <glm/gtc/packing.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/matrix.hpp>
#include <iostream>
#include <sstream>
#include <string>

#include "glad/glad.h"

namespace owds {

  std::string Shader::shaders_directory;

  Shader::Shader(const std::string& vertex_path, const std::string& fragment_path, const std::string& geometry_path)
  {
    // 1. retrieve the vertex/fragment source code from filePath
    std::string vertex_code;
    std::string fragment_code;
    std::ifstream v_shader_file;
    std::ifstream f_shader_file;
    // ensure ifstream objects can throw exceptions:
    v_shader_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    f_shader_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
    {
      // open files
      v_shader_file.open(shaders_directory + vertex_path);
      f_shader_file.open(shaders_directory + fragment_path);
      std::stringstream v_shader_stream, f_shader_stream;
      // read file's buffer contents into streams
      v_shader_stream << v_shader_file.rdbuf();
      f_shader_stream << f_shader_file.rdbuf();
      // close file handlers
      v_shader_file.close();
      f_shader_file.close();
      // convert stream into string
      vertex_code = v_shader_stream.str();
      fragment_code = f_shader_stream.str();
    }
    catch(std::ifstream::failure e)
    {
      std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
    }
    const char* v_shader_code = vertex_code.c_str();
    const char* f_shader_code = fragment_code.c_str();

    // 2. compile shaders
    // vertex Shader
    unsigned int vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &v_shader_code, nullptr);
    glCompileShader(vertex);
    checkCompileErrors(vertex, "VERTEX");

    // similiar for Fragment Shader
    unsigned int fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &f_shader_code, nullptr);
    glCompileShader(fragment);
    checkCompileErrors(fragment, "FRAGMENT");

    unsigned int geometry = 0;
    if(geometry_path.empty() == false)
    {
      std::string geometry_code;
      std::ifstream g_shader_file;
      g_shader_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
      try
      {
        g_shader_file.open(shaders_directory + geometry_path);
        std::stringstream g_shader_stream;
        g_shader_stream << g_shader_file.rdbuf();
        g_shader_file.close();
        geometry_code = g_shader_stream.str();
      }
      catch(std::ifstream::failure e)
      {
        std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
      }
      const char* g_shader_code = geometry_code.c_str();

      // similiar for Geometry Shader
      geometry = glCreateShader(GL_GEOMETRY_SHADER);
      glShaderSource(geometry, 1, &g_shader_code, nullptr);
      glCompileShader(geometry);
      checkCompileErrors(geometry, "GEOMETRY");
    }

    // shader Program
    id_ = glCreateProgram();
    glAttachShader(id_, vertex);
    glAttachShader(id_, fragment);
    if(geometry_path.empty() == false)
      glAttachShader(id_, geometry);
    glLinkProgram(id_);
    checkCompileErrors(id_, "PROGRAM");

    // delete the shaders as they're linked into our program now and no longer necessary
    glDeleteShader(vertex);
    glDeleteShader(fragment);
  }

  void Shader::use()
  {
    glUseProgram(id_);
  }

  void Shader::setBool(const std::string& name, bool value) const
  {
    glUniform1i(glGetUniformLocation(id_, name.c_str()), (int)value);
  }

  void Shader::setInt(const std::string& name, int value) const
  {
    glUniform1i(glGetUniformLocation(id_, name.c_str()), value);
  }

  void Shader::setFloat(const std::string& name, float value) const
  {
    glUniform1f(glGetUniformLocation(id_, name.c_str()), value);
  }

  void Shader::setMat4(const std::string& name, const glm::mat4& value) const
  {
    glUniformMatrix4fv(glGetUniformLocation(id_, name.c_str()), 1, GL_FALSE, glm::value_ptr(value));
  }

  void Shader::setVec3(const std::string& name, const glm::vec3& value) const
  {
    glUniform3fv(glGetUniformLocation(id_, name.c_str()), 1, &value[0]);
  }

  void Shader::setVec4(const std::string& name, const glm::vec4& value) const
  {
    glUniform4fv(glGetUniformLocation(id_, name.c_str()), 1, &value[0]);
  }

  void Shader::checkCompileErrors(GLuint shader, std::string type)
  {
    GLint success = 0;
    GLchar info_log[1024];
    if(type != "PROGRAM")
    {
      glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
      if(success == 0)
      {
        glGetShaderInfoLog(shader, 1024, nullptr, info_log);
        std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n"
                  << info_log << "\n -- --------------------------------------------------- -- " << std::endl;
      }
    }
    else
    {
      glGetProgramiv(shader, GL_LINK_STATUS, &success);
      if(success == 0)
      {
        glGetProgramInfoLog(shader, 1024, nullptr, info_log);
        std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
                  << info_log << "\n -- --------------------------------------------------- -- " << std::endl;
      }
    }
  }

} // namespace owds