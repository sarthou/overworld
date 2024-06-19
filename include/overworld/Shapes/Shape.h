#ifndef OWDS_SHAPES_SHAPE_H
#define OWDS_SHAPES_SHAPE_H

#include <variant>

#include "overworld/Shapes/ShapeBox.h"
#include "overworld/Shapes/ShapeCapsule.h"
#include "overworld/Shapes/ShapeCustomMesh.h"
#include "overworld/Shapes/ShapeCylinder.h"
#include "overworld/Shapes/ShapeDummy.h"
#include "overworld/Shapes/ShapeSphere.h"

namespace owds {
  using Shape = std::variant<
    ShapeBox,
    ShapeCapsule,
    ShapeCustomMesh,
    ShapeCylinder,
    ShapeDummy,
    ShapeSphere>;
}

#endif // OWDS_SHAPES_SHAPE_H
