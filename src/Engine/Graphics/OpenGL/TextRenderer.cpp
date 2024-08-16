#include "overworld/Engine/Graphics/OpenGL/TextRenderer.h"

#include <freetype/freetype.h>
#include <string>

#include "glad/glad.h"
#include "overworld/Engine/Graphics/OpenGL/Shader.h"
#include "overworld/Utility/ShellDisplay.h"

namespace owds {

  void TextRenderer::init()
  {
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 5, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  }

  bool TextRenderer::load(const std::string& font, unsigned int font_size)
  {
    characters_.clear();

    FT_Library ft;
    if(FT_Init_FreeType(&ft) != 0) // all functions return a value different than 0 whenever an error occurred
    {
      ShellDisplay::error("[TextRenderer] Could not init FreeType Library");
      return false;
    }

    FT_Face face;
    if(FT_New_Face(ft, font.c_str(), 0, &face))
    {
      ShellDisplay::error("[TextRenderer] CFailed to load font " + font);
      return false;
    }

    FT_Set_Pixel_Sizes(face, 0, font_size);
    pixel_size_ = font_size;
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    for(GLubyte c = 0; c < 128; c++)
    {
      if(FT_Load_Char(face, c, FT_LOAD_RENDER))
      {
        ShellDisplay::error("[TextRenderer] Failed to load Glyph " + std::string(1, c));
        continue;
      }

      unsigned int texture = 0;
      glGenTextures(1, &texture);
      glBindTexture(GL_TEXTURE_2D, texture);
      glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RED,
        face->glyph->bitmap.width,
        face->glyph->bitmap.rows,
        0,
        GL_RED,
        GL_UNSIGNED_BYTE,
        face->glyph->bitmap.buffer);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

      Character_t character = {
        texture,
        glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
        glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
        (unsigned int)face->glyph->advance.x};
      characters_.emplace((char)c, character);
    }
    glBindTexture(GL_TEXTURE_2D, 0);

    FT_Done_Face(face);
    FT_Done_FreeType(ft);

    return true;
  }

  void TextRenderer::renderText(Shader& shader, const std::string& text, const glm::vec3& position, float height, const glm::vec3& color)
  {
    shader.use();
    shader.setVec3("text_color", color);

    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(vao_);

    height = height / (float)pixel_size_;

    float x = position.x;
    float y = position.y;
    float z = position.z;

    std::string::const_iterator c;
    for(c = text.begin(); c != text.end(); c++)
    {
      Character_t& ch = characters_[*c];

      float xpos = x + ch.bearing.x * height;
      float zpos = z - (ch.size.y - ch.bearing.y) * height;

      float w = ch.size.x * height;
      float h = ch.size.y * height;

      float vertices[6 * 5] = {
        xpos, y, zpos + h, 0.0f, 0.0f,
        xpos, y, zpos, 0.0f, 1.0f,
        xpos + w, y, zpos, 1.0f, 1.0f,

        xpos, y, zpos + h, 0.0f, 0.0f,
        xpos + w, y, zpos, 1.0f, 1.0f,
        xpos + w, y, zpos + h, 1.0f, 0.0f};

      glBindTexture(GL_TEXTURE_2D, ch.texture_id_);
      glBindBuffer(GL_ARRAY_BUFFER, vbo_);
      glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices); // be sure to use glBufferSubData and not glBufferData
      glBindBuffer(GL_ARRAY_BUFFER, 0);

      glDrawArrays(GL_TRIANGLES, 0, 6);

      x += (ch.advance >> 6) * height; // bitshift by 6 to get value in pixels (1/64th times 2^6 = 64)
    }
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

} // namespace owds