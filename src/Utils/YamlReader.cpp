#include "overworld/Utils/YamlReader.h"

#include <cstddef>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <string>
#include <vector>

namespace owds {

  bool YamlReader::read(const std::string& path)
  {
    std::ifstream config_file(path);
    if(config_file.is_open())
    {
      std::vector<std::string> file;
      std::string line;
      while(std::getline(config_file, line))
      {
        removeComment(line);
        file.push_back(line);
      }

      size_t current_line = 0;

      elements_ = read(file, current_line);

      config_file.close();
      return true;
    }
    else
      return false;
  }

  std::map<std::string, YamlElement> YamlReader::read(const std::vector<std::string>& lines, size_t& current_line)
  {
    std::map<std::string, YamlElement> res;

    std::regex element_regex(R"(^\s*([^\s]*)\s*:\s*([^\n]*)\s*$)"); // in case of bug, previous regex was (^\s*([^\s]*)\s*:\s*([^\n\s]*)\s*$)
    std::regex list_regex(R"(^\s*-\s*(.*)\s*$)");
    std::smatch match;

    std::string config_name;

    size_t nb_spaces_base = countSpaces(lines[current_line]);

    do
    {
      if(countSpaces(lines[current_line]) < nb_spaces_base)
        return res;

      if(std::regex_match(lines[current_line], match, element_regex))
      {
        if(match[2].str().empty())
        {
          config_name = match[1].str();
          res[config_name] = YamlElement();
          size_t next_nb_spaces = countSpaces(lines[current_line + 1]);
          if(next_nb_spaces > nb_spaces_base)
          {
            if(std::regex_match(lines[current_line + 1], match, list_regex))
            {
              res[config_name].data_ = std::vector<std::string>();

              do
              {
                current_line++;
                res[config_name].data_->push_back(match[1].str());
              } while((current_line + 1 < lines.size()) && (std::regex_match(lines[current_line + 1], match, list_regex)));
            }
            else
            {
              current_line++;
              res[config_name].subelem_ = read(lines, current_line);
              current_line--;
            }
          }
        }
        else
        {
          config_name = match[1].str();
          res[config_name] = YamlElement();
          res[config_name].data_ = std::vector<std::string>();
          res[config_name].data_->push_back(match[2].str());
        }
      }

      current_line++;
    } while(current_line < lines.size());

    return res;
  }

  void YamlReader::display()
  {
    display(elements_);
  }

  void YamlReader::display(const std::map<std::string, YamlElement>& config, size_t nb)
  {
    if((nb == 0) && (config.find("modules") != config.end()))
      displayElement(*config.find("modules"), nb);

    for(const auto& c : config)
    {
      if((nb == 0) && (c.first == "modules"))
        continue;

      displayElement(c, nb);
    }
  }

  void YamlReader::displayElement(const std::pair<std::string, YamlElement>& it, size_t nb)
  {
    displayTab(nb);
    std::cout << "\e[1m" << it.first << "\e[0m : ";
    if(it.second.data_)
    {
      if(it.second.data_.value().size() > 1)
      {
        std::cout << std::endl;
        for(const auto& d : it.second.data_.value())
        {
          displayTab(nb + 1);
          std::cout << "- " << d << std::endl;
        }
      }
      else
        std::cout << it.second.data_.value().front() << std::endl;
    }
    else if(it.second.subelem_)
    {
      std::cout << std::endl;
      display(it.second.subelem_.value(), nb + 1);
    }
  }

  void YamlReader::displayTab(size_t nb)
  {
    for(size_t i = 0; i < nb; i++)
      std::cout << "\t";
  }

  void YamlReader::removeComment(std::string& line)
  {
    size_t pose = line.find('#');
    if(pose != std::string::npos)
      line = line.substr(0, pose);
  }

  size_t YamlReader::countSpaces(const std::string& line)
  {
    size_t nb = 0;
    while((nb < line.size()) && ((line[nb] == ' ') || (line[nb] == '\t')))
      nb++;

    return nb;
  }

} // namespace owds