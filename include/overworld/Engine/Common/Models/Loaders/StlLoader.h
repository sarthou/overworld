#ifndef OVERWORLD_STLLOADER_H
#define OVERWORLD_STLLOADER_H

#include <memory>
#include <string>

#include "overworld/Engine/Common/Models/Model.h"

namespace owds {

  class StlLoader
  {
  public:
    static std::unique_ptr<owds::Model> read(const std::string& path);
  };

} // namespace owds

#endif // OVERWORLD_STLLOADER_H