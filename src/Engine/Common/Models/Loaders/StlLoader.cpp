#include "overworld/Engine/Common/Models/Loaders/StlLoader.h"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Engine/Common/Models/Vertex.h"
#include "overworld/Utils/ShellDisplay.h"

namespace owds {

  struct Triangle_t
  {
    float normal[3];  // NOLINT
    float vertex0[3]; // NOLINT
    float vertex1[3]; // NOLINT
    float vertex2[3]; // NOLINT
  };

  std::unique_ptr<owds::Model> StlLoader::read(const std::string& path)
  {
    FILE* file = fopen(path.c_str(), "rb");
    if(file == nullptr)
    {
      ShellDisplay::error("[StlLoader] Fail to open file " + path);
      return nullptr;
    }

    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if(file_size <= 0)
    {
      fclose(file);
      return nullptr;
    }

    char* buffer = new char[file_size + 1];
    if(buffer == nullptr)
    {
      ShellDisplay::error("[StlLoader] Memory error while reading " + path);
      return nullptr;
    }

    long result = fread(buffer, 1, file_size, file);
    if(result != file_size)
    {
      ShellDisplay::error("[StlLoader] Fail to read file " + path);
      return nullptr;
    }

    std::unique_ptr<owds::Model> model = nullptr;

    int nb_triangles = *(int*)&buffer[80];
    if(nb_triangles > 0)
    {
      int expected_size = nb_triangles * 50 + 84;
      if(expected_size != file_size)
      {
        delete[] buffer;
        fclose(file);
        ShellDisplay::warning("[StlLoader] No triangle read from " + path);
        return nullptr;
      }

      model = std::make_unique<Model>(Model::create());
      model->source_path_ = path;
      model->scale_ = {1, 1, 1};
      model->meshes_.emplace_back(Mesh::create());
      Mesh* mesh = &(model->meshes_.front());
      mesh->indices_.reserve(3 * nb_triangles);
      mesh->vertices_.reserve(3 * nb_triangles);

      Vertex v0, v1, v2;
      v0.uv_[0] = v1.uv_[0] = v2.uv_[0] = 0.5;
      v0.uv_[1] = v1.uv_[1] = v2.uv_[1] = 0.5;

      int index = 0;
      for(int t = 0; t < nb_triangles; t++)
      {
        char* current = &buffer[84 + t * 50];
        Triangle_t triangle;
        memcpy(&triangle, current, sizeof(Triangle_t));

        for(size_t i = 0; i < 3; i++)
        {
          v0.position_[i] = triangle.vertex0[i];
          v1.position_[i] = triangle.vertex1[i];
          v2.position_[i] = triangle.vertex2[i];
          v0.normal_[i] = v1.normal_[i] = v2.normal_[i] = triangle.normal[i];
        }

        mesh->vertices_.emplace_back(v0);
        mesh->vertices_.emplace_back(v1);
        mesh->vertices_.emplace_back(v2);

        mesh->indices_.emplace_back(index++);
        mesh->indices_.emplace_back(index++);
        mesh->indices_.emplace_back(index++);
      }
    }

    fclose(file);
    delete[] buffer;

    return model;
  }

} // namespace owds