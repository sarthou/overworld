#include "overworld/Utils/Wavefront.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>

namespace owds {

  namespace wavefront {

    bool isWavefront(const std::string& cad_file)
    {
      return (cad_file.find(".obj") != std::string::npos);
    }

    std::vector<std::array<double, 3>> getVertexes(const std::string& obj_path)
    {
      std::vector<std::array<double, 3>> vertexes;
      std::map<int, int> order;
      std::regex v_regex("v\\s([-|\\d|.]+)\\s([-|\\d|.]+)\\s([-|\\d|.]+)");
      std::regex l_regex("l\\s([\\d]+)\\s([\\d]+)");
      std::smatch match;

      std::ifstream obj_file(obj_path);
      if(obj_file.is_open())
      {
        std::string line;
        while(std::getline(obj_file, line))
        {
          if(std::regex_match(line, match, v_regex))
            vertexes.push_back(std::array<double, 3>{std::stod(match[1]), std::stod(match[2]), std::stod(match[3])});
          else if(std::regex_match(line, match, l_regex))
            order[std::stoi(match[1])] = std::stoi(match[2]);
        }

        if(order.size())
        {
          std::vector<std::array<double, 3>> ordered_vertexes;
          int index = order.begin()->first;
          for(size_t i = 0; i < order.size(); i++)
          {
            ordered_vertexes.emplace_back(vertexes[index - 1]);
            index = order[index];
          }
          vertexes = std::move(ordered_vertexes);
        }

        obj_file.close();
      }

      return vertexes;
    }

    std::string getMltFile(const std::string& obj_path)
    {
      std::ifstream obj_f(obj_path);
      std::string obj_content;
      std::string mlt_content;
      if(obj_f)
      {
        std::ostringstream ss;
        ss << obj_f.rdbuf();
        obj_content = ss.str();

        size_t mlt_pose = obj_content.find("mtllib");
        if(mlt_pose != std::string::npos)
        {
          size_t end_pose = obj_content.find("\n", mlt_pose);
          std::string mlt_name = obj_content.substr(mlt_pose + std::string("mtllib ").size(), end_pose - (mlt_pose + std::string("mtllib ").size()));

          std::string mlt_path = obj_path;
          size_t last_pose = mlt_path.find_last_of("/");
          mlt_path.replace(mlt_path.begin() + last_pose, mlt_path.end(), "/" + mlt_name);

          std::ifstream mlt_f(mlt_path);
          if(mlt_f)
          {
            std::ostringstream mlt_ss;
            mlt_ss << mlt_f.rdbuf();
            mlt_content = mlt_ss.str();
          }
        }
      }
      return mlt_content;
    }

    std::vector<std::string> getMltMaterials(const std::string& mlt_content)
    {
      std::vector<std::string> materials;

      size_t newmtl_pose = 0;
      while((newmtl_pose = mlt_content.find("newmtl", newmtl_pose + 1)) != std::string::npos)
      {
        size_t end_pose = mlt_content.find("\n", newmtl_pose);
        std::string material = mlt_content.substr(newmtl_pose + std::string("newmtl ").size(), end_pose - (newmtl_pose + std::string("newmtl ").size()));
        materials.push_back(material);
      }

      return materials;
    }

    std::string getMaterialColor(const std::string& mlt_content, const std::string& material_name)
    {
      std::string color;

      size_t material_pose = mlt_content.find("newmtl " + material_name);
      if(material_pose != std::string::npos)
      {
        size_t kd_pose = mlt_content.find("Kd ", material_pose);
        size_t end_pose = mlt_content.find("\n", kd_pose);
        std::string color_str = mlt_content.substr(kd_pose, end_pose - kd_pose);

        std::regex regex("Kd\\s([\\d|.]+)\\s([\\d|.]+)\\s([\\d|.]+)");
        std::smatch match;
        if(std::regex_match(color_str, match, regex))
        {
          int r = std::stod(match[1].str()) * 255.0;
          int g = std::stod(match[2].str()) * 255.0;
          int b = std::stod(match[3].str()) * 255.0;
          std::stringstream stream;
          stream << std::setfill('0') << std::setw(2) << std::hex << r << std::setfill('0') << std::setw(2) << g << std::setfill('0') << std::setw(2) << b;
          return stream.str();
        }
      }

      return color;
    }

    std::string getMaterialTexture(const std::string& mlt_content, const std::string& material_name)
    {
      std::string texture;

      size_t material_pose = mlt_content.find("newmtl " + material_name);
      if(material_pose != std::string::npos)
      {
        size_t next_material = mlt_content.find("newmtl ", material_pose + 1);
        size_t kd_pose = mlt_content.find("map_Kd ", material_pose);
        if(kd_pose != std::string::npos)
        {
          if((next_material == std::string::npos) || (kd_pose < next_material))
          {
            size_t end_pose = mlt_content.find("\n", kd_pose);
            if(end_pose != std::string::npos)
              texture = mlt_content.substr(kd_pose + std::string("map_Kd ").size(), end_pose - (kd_pose + std::string("map_Kd ").size()));
            else
              texture = mlt_content.substr(kd_pose + std::string("map_Kd ").size());
          }
        }
      }

      return texture;
    }

  } // namespace wavefront

} // namespace owds