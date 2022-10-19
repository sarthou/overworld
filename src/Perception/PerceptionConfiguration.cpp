#include "overworld/Perception/PerceptionConfiguration.h"

#include <fstream>
#include <iostream>
#include <regex>

namespace owds
{

bool PerceptionConfiguration::read(const std::string& path)
{
  std::ifstream config_file(path);
  if(config_file.is_open())
  {

    std::vector<std::string> file;
    std::string line;
    while(std::getline(config_file,line))
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

std::map<std::string, ConfigElement> PerceptionConfiguration::read(const std::vector<std::string>& lines, size_t& current_line)
{
  std::map<std::string, ConfigElement> res;

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
      if(match[2].str() == "")
      {
        config_name = match[1].str();
        res[config_name] = ConfigElement();
        size_t next_nb_spaces = countSpaces(lines[current_line + 1]);
        if(next_nb_spaces > nb_spaces_base)
        {
          if(std::regex_match(lines[current_line + 1], match, list_regex))
          {
            res[config_name].data = std::vector<std::string>();

            do
            {
              current_line++;
              res[config_name].data->push_back(match[1].str());
            }
            while((current_line + 1 < lines.size()) && (std::regex_match(lines[current_line + 1], match, list_regex)));
          }
          else
          {
            current_line++;
            res[config_name].subelem = read(lines, current_line);
            current_line--;
          }
        }
      }
      else
      {
        config_name = match[1].str();
        res[config_name] = ConfigElement();
        res[config_name].data = std::vector<std::string>();
        res[config_name].data->push_back(match[2].str());
      }
    }

    current_line++;
  }
  while(current_line < lines.size());

  return res;
}

void PerceptionConfiguration::display()
{
  display(elements_);
}

void PerceptionConfiguration::display(std::map<std::string, ConfigElement>& config, size_t nb)
{
  for(auto& c : config)
  {
    displayTab(nb);
    std::cout << "\e[1m" << c.first << "\e[0m : ";
    if(c.second.data)
    {
      if(c.second.data.value().size() > 1)
      {
        std::cout << std::endl;
        for(auto& d : c.second.data.value())
        {
          displayTab(nb+1);
          std::cout << "- " << d << std::endl;
        }
      }
      else
        std::cout << c.second.data.value().front() << std::endl;
    }
    else if(c.second.subelem)
    {
      std::cout << std::endl;
      display(c.second.subelem.value(), nb+1);
    }
  }
}

void PerceptionConfiguration::displayTab(size_t nb)
{
  for(size_t i = 0; i < nb; i++)
    std::cout << "\t";
}

void PerceptionConfiguration::removeComment(std::string& line)
{
  size_t pose = line.find('#');
  if(pose != std::string::npos)
    line = line.substr(0, pose);
}

size_t PerceptionConfiguration::countSpaces(const std::string& line)
{
  size_t nb = 0;
  while((nb < line.size()) && ((line[nb] == ' ') || (line[nb] == '\t')) )
    nb++;

  return nb;
}

} // namespace owds
