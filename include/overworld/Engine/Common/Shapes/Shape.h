#ifndef OWDS_COMMON_SHAPE_H
#define OWDS_COMMON_SHAPE_H

#include <variant>

#include "overworld/Engine/Common/Shapes/ShapeBox.h"
#include "overworld/Engine/Common/Shapes/ShapeCapsule.h"
#include "overworld/Engine/Common/Shapes/ShapeCustomMesh.h"
#include "overworld/Engine/Common/Shapes/ShapeCylinder.h"
#include "overworld/Engine/Common/Shapes/ShapeDummy.h"
#include "overworld/Engine/Common/Shapes/ShapeSkybox.h"
#include "overworld/Engine/Common/Shapes/ShapeSphere.h"

namespace owds {
  using Shape = std::variant<
    ShapeBox,
    ShapeCapsule,
    ShapeCustomMesh,
    ShapeCylinder,
    ShapeDummy,
    // ShapeSkybox,
    ShapeSphere>;
}

#endif // OWDS_COMMON_SHAPE_H
