#ifndef OWDS_YAMLREADER_H
#define OWDS_YAMLREADER_H

#include <algorithm>
#include <experimental/optional>
#include <map>
#include <string>
#include <vector>

namespace owds {

  class YamlReader;

  class YamlElement
  {
    friend YamlReader;

  public:
    YamlElement operator[](const std::string& name)
    {
      if(subelem)
      {
        if(subelem.value().find(name) != subelem.value().end())
          return subelem.value()[name];
        else
          return YamlElement();
      }
      else
        return YamlElement();
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
        std::transform(subelem.value().cbegin(), subelem.value().cend(),
                       std::back_inserter(res),
                       [](const std::pair<std::string, YamlElement>& elem) { return elem.first; });
      return res;
    }

    bool keyExists(const std::string& key)
    {
      if(subelem)
        return (subelem.value().find(key) != subelem.value().end());
      else
        return false;
    }

  private:
    std::experimental::optional<std::vector<std::string>> data;
    std::experimental::optional<std::map<std::string, YamlElement>> subelem;
  };

  class YamlReader
  {
  public:
    bool read(const std::string& path);

    void display();

    YamlElement operator[](const std::string& name)
    {
      if(elements_.find(name) != elements_.end())
        return elements_[name];
      else
        return YamlElement();
    }

    std::vector<std::string> getKeys()
    {
      std::vector<std::string> res;
      std::transform(elements_.cbegin(), elements_.cend(),
                     std::back_inserter(res),
                     [](const std::pair<std::string, YamlElement>& elem) { return elem.first; });
      return res;
    }

    bool keyExists(const std::string& key)
    {
      return (elements_.find(key) != elements_.end());
    }

  private:
    std::map<std::string, YamlElement> elements_;

    std::map<std::string, YamlElement> read(const std::vector<std::string>& lines, size_t& current_line);
    YamlElement readList(const std::vector<std::string>& lines, size_t& current_line);

    void display(const std::map<std::string, YamlElement>& config, size_t nb = 0);
    void displayElement(const std::pair<std::string, YamlElement>& it, size_t nb);
    void displayTab(size_t nb);
    void removeComment(std::string& line);
    size_t countSpaces(const std::string& line);
  };

} // namespace owds

#endif // OWDS_YAMLREADER_H
