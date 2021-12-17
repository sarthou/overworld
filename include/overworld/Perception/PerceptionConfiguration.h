#ifndef OWDS_PERCEPTIONCONFIGURATION_H
#define OWDS_PERCEPTIONCONFIGURATION_H

#include <string>
#include <vector>
#include <map>
#include <experimental/optional>

namespace owds
{

class PerceptionConfiguration;

class ConfigElement
{
  friend PerceptionConfiguration;
public:
  ConfigElement operator[](const std::string& name)
  {
    if(subelem)
    {
      if(subelem.value().find(name) != subelem.value().end())
        return subelem.value()[name];
      else
        return ConfigElement();
    }
    else
      return ConfigElement();
  }

  std::vector<std::string> value()
  {
    if(data)
      return data.value();
    else
      return {};
  }

  std::vector<std::string> getElementsKeys()
  {
    std::vector<std::string> res;
    if(subelem)
      for(auto& elem : subelem.value())
        res.push_back(elem.first);
    return res;
  }

private:
  std::experimental::optional<std::vector<std::string>> data;
  std::experimental::optional<std::map<std::string, ConfigElement>> subelem;
};

class PerceptionConfiguration
{
public:
  bool read(const std::string& path);

  void display();

  ConfigElement operator[](const std::string& name)
  {
    if(elements_.find(name) != elements_.end())
      return elements_[name];
    else
      return ConfigElement();
  }

private:
  std::map<std::string, ConfigElement> elements_;

  std::map<std::string, ConfigElement> read(const std::vector<std::string>& lines, size_t& current_line);
  ConfigElement readList(const std::vector<std::string>& lines, size_t& current_line);

  void display(std::map<std::string, ConfigElement>& config, size_t nb = 0);
  void displayTab(size_t nb);
  void removeComment(std::string& line);
  size_t countSpaces(const std::string& line);
};

} // namespace owds

#endif // OWDS_PERCEPTIONCONFIGURATION_H
