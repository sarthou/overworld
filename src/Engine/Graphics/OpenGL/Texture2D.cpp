#include "overworld/Engine/Graphics/OpenGL/Texture2D.h"

#include <array>
#include <iostream>
#include <string>

#include "glad/glad.h"

#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#include "stb_image.h"
#endif

namespace owds {
  Texture2D::Texture2D(const std::string& path,
                       TextureType_e type,
                       bool gamma_correction, bool flip) : type_(type)
  {
    glGenTextures(1, &id_);

    stbi_set_flip_vertically_on_load(flip);
    unsigned char* data = stbi_load(path.c_str(), &width, &height, &nb_channels, 0);
    if(data != nullptr)
    {
      if(nb_channels == 1)
      {
        internal_format = data_format = GL_RED;
        if(type == texture_diffuse)
        {
          loadGreyAsRgb(data);
          return;
        }
      }
      else if(nb_channels == 3)
      {
        internal_format = gamma_correction ? GL_SRGB : GL_RGB;
        data_format = GL_RGB;
      }
      else if(nb_channels == 4)
      {
        internal_format = gamma_correction ? GL_SRGB_ALPHA : GL_RGBA;
        data_format = GL_RGBA;
      }

      glBindTexture(GL_TEXTURE_2D, id_);
      glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 0, data_format, GL_UNSIGNED_BYTE, data);
      glGenerateMipmap(GL_TEXTURE_2D);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, internal_format == GL_SRGB_ALPHA ? GL_CLAMP_TO_EDGE : GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, internal_format == GL_SRGB_ALPHA ? GL_CLAMP_TO_EDGE : GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // GL_NEAREST

      stbi_image_free(data);
    }
    else
    {
      std::cout << "Texture2D failed to load at path: " << path << std::endl;
      stbi_image_free(data);
    }
  }

  Texture2D::Texture2D(const std::array<uint8_t, 4>& color, TextureType_e type) : type_(type),
                                                                                  width(1),
                                                                                  height(1),
                                                                                  nb_channels(1),
                                                                                  internal_format(GL_RGBA),
                                                                                  data_format(GL_RGBA)
  {
    glGenTextures(1, &id_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 10, 10, 0, GL_RGBA, GL_UNSIGNED_BYTE, color.data());
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    std::cout << "color texture ceated " << id_ << std::endl;
  }

  void Texture2D::loadGreyAsRgb(unsigned char* data)
  {
    unsigned char* image = new unsigned char[width * height * 3];
    for(int i = 0; i < height; i++)
    {
      for(int j = 0; j < width; j++)
      {
        image[3 * (i * width + j) + 0] = data[(i * width + j)];
        image[3 * (i * width + j) + 1] = data[(i * width + j)];
        image[3 * (i * width + j) + 2] = data[(i * width + j)];
      }
    }

    glBindTexture(GL_TEXTURE_2D, id_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // GL_NEAREST

    delete[] image;
    stbi_image_free(data);
  }

} // namespace owds