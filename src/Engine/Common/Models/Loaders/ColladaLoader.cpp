#include "overworld/Engine/Common/Models/Loaders/ColladaLoader.h"

#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <map>
#include <memory>
#include <string>
#include <tinyxml.h>
#include <vector>

#include "overworld/Engine/Common/Models/Mesh.h"
#include "overworld/Engine/Common/Models/Model.h"
#include "overworld/Utility/ShellDisplay.h"

namespace owds {

  struct EffectParam_t
  {
    std::string type;
    std::string source;
    std::string init_from;
  };

  struct VertexSource_t
  {
    std::string position_id;
    std::string normal_id;
  };

  struct TokenFloatArray_t
  {
    std::vector<float>& values;
    TokenFloatArray_t(std::vector<float>& float_array) : values(float_array)
    {}

    void add(const char* token)
    {
      float v = atof(token);
      values.push_back(v);
    }
  };
  struct TokenIntArray_t
  {
    std::vector<int>& values;
    TokenIntArray_t(std::vector<int>& array) : values(array)
    {}

    void add(const char* token)
    {
      float v = atoi(token);
      values.push_back(v);
    }
  };

  template<typename AddToken>
  void tokenize(const std::string& str, AddToken& token_adder, const std::string& delimiters = " \n")
  {
    std::string::size_type pos;
    std::string::size_type last_pos = 0;
    while(true)
    {
      pos = str.find_first_of(delimiters, last_pos);
      if(pos == std::string::npos)
      {
        pos = str.length();
        if(pos != last_pos)
          token_adder.add(str.data() + last_pos);

        break;
      }
      else if(pos != last_pos)
        token_adder.add(str.data() + last_pos);

      last_pos = pos + 1;
    }
  }

  glm::vec3 getVector3FromXmlText(const char* text)
  {
    glm::vec3 vec(0, 0, 0);
    std::vector<float> float_array;
    TokenFloatArray_t adder(float_array);
    float_array.reserve(3);
    tokenize(text, adder);
    assert(float_array.size() == 3);

    vec = glm::vec3(float_array[0], float_array[1], float_array[2]);

    return vec;
  }

  glm::vec4 getVector4FromXmlText(const char* text)
  {
    glm::vec4 vec(0, 0, 0, 0);
    std::vector<float> float_array;
    TokenFloatArray_t adder(float_array);
    float_array.reserve(4);
    tokenize(text, adder);
    assert(float_array.size() == 4);

    vec = glm::vec4(float_array[0], float_array[1], float_array[2], float_array[3]);

    return vec;
  }

  void getColorAndTexture(TiXmlElement* elem, std::map<std::string, EffectParam_t>& params, std::map<std::string, std::string>& images, Color& color, std::string& texture)
  {
    auto* color_elem = elem->FirstChildElement("color");
    if(color_elem != nullptr)
    {
      auto color_vec = getVector4FromXmlText(color_elem->GetText());
      color.r_ = color_vec.x;
      color.g_ = color_vec.y;
      color.b_ = color_vec.z;
      color.a_ = color_vec.w;
      return;
    }

    auto* texture_elem = elem->FirstChildElement("texture");
    if(texture_elem != nullptr)
    {
      auto* texture_txt = texture_elem->Attribute("texture");
      auto sampler_it = params.find(texture_txt);
      if(sampler_it != params.end())
      {
        auto surface_it = params.find(sampler_it->second.source);
        if(surface_it != params.end())
          texture = surface_it->second.init_from;
        else
          texture = sampler_it->second.source;
      }
      else
        texture = texture_txt;

      if(texture.empty() == false)
      {
        auto image_it = images.find(texture);
        if(image_it != images.end())
          texture = image_it->second;
        else
          texture = "";
      }
      return;
    }
  }

  std::unique_ptr<owds::Model> ColladaLoader::read(const std::string& path)
  {
    TiXmlDocument doc;
    if(getXmlDocument(path, doc) == false)
      return nullptr;

    getScalingAndTransform(doc.RootElement());

    auto mesh_library = getMeshLibrary(doc.RootElement());
    auto material_library = getMaterialLibrary(doc.RootElement());
    (void)material_library;
    auto instances = readSceneGeometries(doc.RootElement(), mesh_library);

    for(auto& material : material_library)
    {
      std::cout << material.first << ":" << std::endl;
      std::cout << " -> diffuse = " << material.second.diffuse_color_.r_ << " " << material.second.diffuse_color_.g_ << " " << material.second.diffuse_color_.b_ << " " << material.second.diffuse_color_.a_ << std::endl;
      std::cout << " -> diffuse = " << material.second.diffuse_texture_ << std::endl;
      std::cout << " -> specular = " << material.second.specular_color_.r_ << " " << material.second.specular_color_.g_ << " " << material.second.specular_color_.b_ << " " << material.second.specular_color_.a_ << std::endl;
      std::cout << " -> specular = " << material.second.specular_texture_ << std::endl;
      std::cout << " -> normal   = " << material.second.normal_texture_ << std::endl;
      std::cout << " -> shininess = " << material.second.shininess_ << std::endl;
    }

    if(instances.empty() == false)
    {
      auto model = std::make_unique<Model>(Model::create());
      model->source_path_ = path;
      model->meshes_.swap(instances);
      return model;
    }

    return nullptr;
  }

  bool ColladaLoader::getXmlDocument(const std::string& path, TiXmlDocument& doc)
  {
    std::string content;
    std::ifstream f(path);

    if(!f.is_open())
    {
      ShellDisplay::error("Fail to open : " + path);
      return false;
    }

    std::string tmp;
    while(std::getline(f, tmp))
      content += tmp;

    // removeDocType(response);

    doc.Parse((const char*)content.c_str(), nullptr, TIXML_ENCODING_UTF8);

    TiXmlElement* root = doc.RootElement();
    if(root == nullptr)
    {
      return false;
    }

    return true;
  }

  void ColladaLoader::getScalingAndTransform(TiXmlElement* root)
  {
    unit_meter_scaling_ = 1.0;
    tr_ = glm::mat3(1.0);

    auto* asset = root->FirstChildElement("asset");
    if(asset != nullptr)
    {
      auto* unit = asset->FirstChildElement("unit");
      if(unit != nullptr)
      {
        const char* meter_text = unit->Attribute("meter");
        unit_meter_scaling_ = std::atof(meter_text);
      }

      auto* up_axis = asset->FirstChildElement("up_axis");
      if(up_axis != nullptr)
      {
        std::string up_text = up_axis->GetText();
        if(up_text == "X_UP")
          tr_ = glm::mat3(glm::rotate(glm::mat4(1.0f), glm::radians(-90.f), glm::vec3(0, 1, 0)));
        if(up_text == "Y_UP")
          tr_ = glm::mat3(glm::rotate(glm::mat4(1.0f), glm::radians(90.f), glm::vec3(1, 0, 0)));
        // if(up_text == "Z_UP") // client and COLLADA are both Z_UP so no transform needed (identity)
      }
    }
  }

  std::map<std::string, Material> ColladaLoader::getMaterialLibrary(TiXmlElement* root)
  {
    std::map<std::string, Material> res;

    std::map<std::string, std::string> images;
    auto* image_library = root->FirstChildElement("library_images");
    if(image_library != nullptr)
    {
      for(auto* image = image_library->FirstChildElement("image"); image != nullptr; image = image->NextSiblingElement("image"))
      {
        std::string image_id = std::string(image->Attribute("id"));
        auto* init_from = image->FirstChildElement("init_from");
        if(init_from != nullptr)
          images.emplace(image_id, init_from->GetText());
      }
    }

    std::map<std::string, Material> effects;
    auto* effect_library = root->FirstChildElement("library_effects");
    if(effect_library != nullptr)
    {
      for(auto* effect = effect_library->FirstChildElement("effect"); effect != nullptr; effect = effect->NextSiblingElement("effect"))
      {
        std::string effect_id = std::string(effect->Attribute("id"));
        auto* profile_common = effect->FirstChildElement("profile_COMMON");
        if(profile_common != nullptr)
        {
          std::map<std::string, EffectParam_t> params;
          for(auto* param = profile_common->FirstChildElement("newparam"); param != nullptr; param = param->NextSiblingElement("newparam"))
          {
            std::string sid = std::string(param->Attribute("sid"));
            auto* surface = param->FirstChildElement("surface");
            if(surface != nullptr)
            {
              params.emplace(sid, EffectParam_t{"surface", "", surface->FirstChildElement("init_from")->GetText()});
              continue;
            }

            auto* sampler = param->FirstChildElement("sampler2D");
            if(sampler != nullptr)
            {
              params.emplace(sid, EffectParam_t{"sampler2D", sampler->FirstChildElement("source")->GetText(), ""});
              continue;
            }
          }

          auto* technique = profile_common->FirstChildElement("technique");
          if(technique != nullptr)
          {
            Material material;

            auto* extra = technique->FirstChildElement("extra");
            if(extra != nullptr)
            {
              auto* extra_technique = extra->FirstChildElement("technique");
              if(extra_technique != nullptr)
              {
                Color tmp;
                auto* bump = extra_technique->FirstChildElement("bump");
                if(bump != nullptr)
                  getColorAndTexture(bump, params, images, tmp, material.normal_texture_);
              }
            }

            auto* blinn = technique->FirstChildElement("blinn");
            if(blinn != nullptr)
            {
              auto* diffuse = blinn->FirstChildElement("diffuse");
              if(diffuse != nullptr)
                getColorAndTexture(diffuse, params, images, material.diffuse_color_, material.diffuse_texture_);

              auto* specular = blinn->FirstChildElement("specular");
              if(specular != nullptr)
                getColorAndTexture(specular, params, images, material.specular_color_, material.specular_texture_);

              auto* shininess = blinn->FirstChildElement("shininess");
              if(shininess != nullptr)
                material.shininess_ = atof(shininess->FirstChildElement("float")->GetText());

              effects.emplace(effect_id, material);

              continue;
            }

            auto* lambert = technique->FirstChildElement("lambert");
            if(lambert != nullptr)
            {
              auto* diffuse = lambert->FirstChildElement("diffuse");
              if(diffuse != nullptr)
                getColorAndTexture(diffuse, params, images, material.diffuse_color_, material.diffuse_texture_);

              effects.emplace(effect_id, material);
              continue;
            }
          }
        }
        else
          ShellDisplay::warning("[ColladaLoader] no profile_COMMON found for effect effect_id. Effect will be ignored.");
      }
    }

    auto* material_library = root->FirstChildElement("library_materials");
    if(material_library == nullptr)
      return {};

    for(auto* material = material_library->FirstChildElement("material"); material != nullptr; material = material->NextSiblingElement("material"))
    {
      std::string material_id = std::string(material->Attribute("id"));
      auto* instance_effect = material->FirstChildElement("instance_effect");
      if(instance_effect != nullptr)
      {
        std::string effect_url(instance_effect->Attribute("url"));
        auto effect_it = effects.find(effect_url.erase(0, 1));
        if(effect_it != effects.end())
          res.emplace(material_id, effect_it->second);
      }
    }

    return res;
  }

  std::map<std::string, Mesh> ColladaLoader::getMeshLibrary(TiXmlElement* root)
  {
    std::map<std::string, Mesh> res;
    std::map<std::string, TiXmlElement*> all_sources;
    std::map<std::string, VertexSource_t> vertex_sources;

    auto* library = root->FirstChildElement("library_geometries");
    if(library == nullptr)
      return {};

    for(auto* geometry = library->FirstChildElement("geometry"); geometry != nullptr; geometry = geometry->NextSiblingElement("geometry"))
    {
      std::vector<glm::vec3> vertex_positions;
      std::vector<glm::vec3> vertex_normals;
      std::vector<glm::vec2> vertex_uvs;
      std::vector<uint32_t> indices;

      std::string mesh_id = std::string(geometry->Attribute("id"));

      for(auto* mesh_elem = geometry->FirstChildElement("mesh"); mesh_elem != nullptr; mesh_elem = mesh_elem->NextSiblingElement("mesh"))
      {
        for(auto* source = mesh_elem->FirstChildElement("source"); source != nullptr; source = source->NextSiblingElement("source"))
          all_sources.emplace(std::string(source->Attribute("id")), source);

        auto* vertices_elem = mesh_elem->FirstChildElement("vertices");
        VertexSource_t vs;
        for(auto* input = vertices_elem->FirstChildElement("input"); input != nullptr; input = input->NextSiblingElement("input"))
        {
          std::string sem_name(input->Attribute("semantic"));
          std::string source_name = std::string(input->Attribute("source"));
          source_name = source_name.erase(0, 1); // remove the #
          if(sem_name == "POSITION")
            vs.position_id = source_name;
          else if(sem_name == "NORMAL")
            vs.normal_id = source_name;
        }
        vertex_sources.emplace(std::string(vertices_elem->Attribute("id")), vs);

        std::vector<TiXmlElement*> triangles_and_polylists;

        for(auto* primitive = mesh_elem->FirstChildElement("triangles"); primitive != nullptr; primitive = primitive->NextSiblingElement("triangles"))
          triangles_and_polylists.push_back(primitive);

        for(auto* primitive = mesh_elem->FirstChildElement("polylist"); primitive != nullptr; primitive = primitive->NextSiblingElement("polylist"))
          triangles_and_polylists.push_back(primitive);

        for(auto* primitive : triangles_and_polylists)
        {
          int primitive_count = 0;
          primitive->QueryIntAttribute("count", &primitive_count);

          std::string vertex_input_name;
          std::string normal_input_name;
          std::string texcoord_input_name;

          int index_stride = 1;
          int vertex_offset = 0;
          int normal_offset = 0;
          int texcoord_offset = 0;

          for(auto* input = primitive->FirstChildElement("input"); input != nullptr; input = input->NextSiblingElement("input"))
          {
            int offset = atoi(input->Attribute("offset"));
            if((offset + 1) > index_stride)
              index_stride = offset + 1;

            std::string source_name = std::string(input->Attribute("source")).erase(0, 1);
            std::string sem_name(input->Attribute("semantic"));
            if(sem_name == "VERTEX")
            {
              // now we have POSITION and possibly NORMAL too, using same index array (<p>)
              VertexSource_t& vs = vertex_sources[source_name];
              if(vs.position_id.empty() == false)
              {
                vertex_input_name = vs.position_id;
                vertex_offset = offset;
              }
              if(vs.normal_id.empty() == false)
              {
                normal_input_name = vs.normal_id;
                normal_offset = offset;
              }
            }
            else if(sem_name == "NORMAL")
            {
              normal_input_name = source_name;
              normal_offset = offset;
            }
            else if(sem_name == "TEXCOORD")
            {
              texcoord_input_name = source_name;
              texcoord_offset = offset;
            }
          }
          int num_indices = primitive_count * 3; // already triangulated

          std::vector<float> position_array;
          int pos_stride = 1;
          auto source_it = all_sources.find(vertex_input_name);
          if(source_it != all_sources.end())
            readFloatArray(source_it->second, position_array, pos_stride);

          std::vector<float> normal_array;
          int normal_stride = 1;
          source_it = all_sources.find(normal_input_name);
          if(source_it != all_sources.end())
            readFloatArray(source_it->second, normal_array, normal_stride);

          std::vector<float> texcoord_array;
          int texcoord_stride = 1;
          source_it = all_sources.find(texcoord_input_name);
          if(source_it != all_sources.end())
            readFloatArray(source_it->second, texcoord_array, texcoord_stride);

          std::vector<int> cur_indices;
          cur_indices.reserve(num_indices * index_stride);
          TokenIntArray_t adder(cur_indices);
          std::string txt = primitive->FirstChildElement("p")->GetText();
          tokenize(txt, adder);
          assert((int)cur_indices.size() == num_indices * index_stride);

          int index_offset = vertex_positions.size();

          for(int index = 0; index < num_indices; index++)
          {
            int pos_index = cur_indices[index * index_stride + vertex_offset];
            int normal_index = cur_indices[index * index_stride + normal_offset];
            int texcoord_index = cur_indices[index * index_stride + texcoord_offset];

            vertex_positions.emplace_back(position_array[pos_index * 3 + 0],
                                          position_array[pos_index * 3 + 1],
                                          position_array[pos_index * 3 + 2]);

            if(normal_array.size() && ((int)normal_array.size() > normal_index))
            {
              vertex_normals.emplace_back(normal_array[normal_index * 3 + 0],
                                          normal_array[normal_index * 3 + 1],
                                          normal_array[normal_index * 3 + 2]);
            }
            else // add a dummy normal of length zero, so it is easy to detect that it is an invalid normal
              vertex_normals.emplace_back(0, 0, 0);

            if(texcoord_array.size() && ((int)texcoord_array.size() > texcoord_index))
            {
              vertex_uvs.emplace_back(texcoord_array[texcoord_index * 2 + 0],
                                      texcoord_array[texcoord_index * 2 + 1]);
            }
            else
              vertex_uvs.emplace_back(0.5, 0.5);
          }

          int cur_num_indices = indices.size();
          indices.resize(cur_num_indices + num_indices);
          for(int index = 0; index < num_indices; index++)
            indices[cur_num_indices + index] = index + index_offset;

        } // if(primitive != nullptr)
      } // for each mesh_elem

      Mesh& mesh = res.emplace(mesh_id, Mesh::create()).first->second;

      assert(vertex_normals.size() == vertex_positions.size());
      assert(vertex_uvs.size() == vertex_positions.size());
      mesh.vertices_.resize(vertex_positions.size());
      for(std::vector<uint32_t>::size_type v = 0; v < vertex_positions.size(); v++)
      {
        mesh.vertices_[v].position_ = vertex_positions[v];
        mesh.vertices_[v].normal_ = vertex_normals[v];
        mesh.vertices_[v].uv_ = vertex_uvs[v];
      }

      mesh.indices_ = indices;
      mesh.name_ = mesh_id;

    } // for each geometry

    return res;
  }

  std::vector<Mesh> ColladaLoader::readSceneGeometries(TiXmlElement* root, std::map<std::string, Mesh>& meshes_library)
  {
    std::vector<Mesh> res;
    std::map<std::string, TiXmlElement*> all_instances;

    TiXmlElement* visual_scenes_elem = root->FirstChildElement("library_visual_scenes");
    if(visual_scenes_elem == nullptr)
      return {};

    for(TiXmlElement* scene = visual_scenes_elem->FirstChildElement("visual_scene"); scene != nullptr; scene = scene->NextSiblingElement("visual_scene"))
      all_instances.emplace(std::string(scene->Attribute("id")), scene);

    TiXmlElement* scene = nullptr;

    TiXmlElement* scenes = root->FirstChildElement("scene");
    if(scenes != nullptr)
    {
      TiXmlElement* instance_scene_reference = scenes->FirstChildElement("instance_visual_scene");
      if(instance_scene_reference != nullptr)
      {
        std::string instance_scene_url = std::string(instance_scene_reference->Attribute("url")).erase(0, 1);
        auto instance_it = all_instances.find(instance_scene_url);
        if(instance_it != all_instances.end())
          scene = instance_it->second;
      }
    }

    if(scene != nullptr)
    {
      for(TiXmlElement* node = scene->FirstChildElement("node"); node != nullptr; node = node->NextSiblingElement("node"))
      {
        glm::mat4 identity(tr_);
        readNodeHierarchy(node, meshes_library, res, identity);
      }
    }

    return res;
  }

  void ColladaLoader::readNodeHierarchy(TiXmlElement* node, std::map<std::string, Mesh>& meshes_library, std::vector<Mesh>& instances, const glm::mat4& parent_trans_mat)
  {
    glm::mat4 node_trans(1.);

    for(TiXmlElement* trans_elem = node->FirstChildElement("matrix"); trans_elem != nullptr; trans_elem = node->NextSiblingElement("matrix"))
    {
      if(trans_elem->GetText() != nullptr)
      {
        std::vector<float> float_array;
        TokenFloatArray_t adder(float_array);
        tokenize(trans_elem->GetText(), adder);
        if(float_array.size() == 16)
        {
          glm::mat4 t(float_array[0], float_array[1], float_array[2], float_array[3],
                      float_array[4], float_array[5], float_array[6], float_array[7],
                      float_array[8], float_array[9], float_array[10], float_array[11],
                      float_array[12], float_array[13], float_array[14], float_array[15]);

          node_trans = node_trans * t;
        }
        else
          ShellDisplay::error("[COlladaLoader] expected 16 elements in a <matrix> element, skipping");
      }
    }

    for(TiXmlElement* trans_elem = node->FirstChildElement("translate"); trans_elem != nullptr; trans_elem = node->NextSiblingElement("translate"))
    {
      if(trans_elem->GetText() != nullptr)
      {
        glm::vec3 pos = getVector3FromXmlText(trans_elem->GetText());
        node_trans = glm::translate(node_trans, pos);
      }
    }

    for(TiXmlElement* scale_elem = node->FirstChildElement("scale"); scale_elem != nullptr; scale_elem = node->NextSiblingElement("scale"))
    {
      if(scale_elem->GetText() != nullptr)
      {
        glm::vec3 scaling = getVector3FromXmlText(scale_elem->GetText());
        node_trans = glm::scale(node_trans, scaling);
      }
    }

    for(TiXmlElement* rotate_elem = node->FirstChildElement("rotate"); rotate_elem != nullptr; rotate_elem = node->NextSiblingElement("rotate"))
    {
      if(rotate_elem->GetText() != nullptr)
      {
        // accumulate orientation
        glm::vec4 rotate = getVector4FromXmlText(rotate_elem->GetText());
        node_trans = glm::rotate(node_trans, glm::radians(rotate[3]), glm::vec3(rotate));
      }
    }

    node_trans = parent_trans_mat * node_trans;

    for(TiXmlElement* instance_geom = node->FirstChildElement("instance_geometry"); instance_geom != nullptr; instance_geom = instance_geom->NextSiblingElement("instance_geometry"))
    {
      std::string geom_url = std::string(instance_geom->Attribute("url")).erase(0, 1);
      auto geom_it = meshes_library.find(geom_url);
      if(geom_it != meshes_library.end())
      {
        instances.emplace_back(geom_it->second);
        Mesh& instance = instances.back();
        for(auto& v : instance.vertices_)
          v.position_ = glm::vec3(node_trans * glm::vec4(v.position_, 1.));

        // TODO get bind_material
      }
      else
        ShellDisplay::error("[ColladaLoader] geom " + geom_url + " not found");
    }

    for(TiXmlElement* child_node = node->FirstChildElement("node"); child_node != nullptr; child_node = child_node->NextSiblingElement("node"))
      readNodeHierarchy(child_node, meshes_library, instances, node_trans);
  }

  void ColladaLoader::readFloatArray(TiXmlElement* source, std::vector<float>& float_array, int& component_stride)
  {
    TiXmlElement* array = source->FirstChildElement("float_array");
    if(array != nullptr)
    {
      int stride = 0;
      component_stride = 1;
      if(source->FirstChildElement("technique_common")->FirstChildElement("accessor")->QueryIntAttribute("stride", &stride) != TIXML_NO_ATTRIBUTE)
        component_stride = stride;

      int count = 0;
      array->QueryIntAttribute("count", &count);
      float_array.reserve(count);

      std::string txt = array->GetText();
      TokenFloatArray_t adder(float_array);
      tokenize(array->GetText(), adder);

      assert((int)float_array.size() == count);
    }
  }

} // namespace owds