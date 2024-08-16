#ifndef OWDS_TEXTRENDERER_H
#define OWDS_TEXTRENDERER_H

#include <glm/matrix.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <map>
#include <string>

#include "overworld/Engine/Common/Debug/DebugText.h"

namespace owds {

  class Shader;

  struct Character_t
  {
    unsigned int texture_id_;
    glm::ivec2 size;      // size of glyph
    glm::ivec2 bearing;   // offset from baseline to left/top of glyph
    unsigned int advance; // horizontal offset to advance to next glyph
  };

  class TextRenderer
  {
  public:
    void init();

    bool load(const std::string& font, unsigned int font_size);

    void renderText(Shader& shader, const glm::mat4& view_matrix, const std::string& text, const glm::vec3& position, float height, const glm::vec3& color, bool center = false);
    void renderText(Shader& shader, const glm::mat4& view_matrix, const DebugText_t& text);

  private:
    unsigned int vao_;
    unsigned int vbo_;
    unsigned int pixel_size_;

    std::map<char, Character_t> characters_;
  };

} // namespace owds

#endif // OWDS_TEXTRENDERER_H