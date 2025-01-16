#include "overworld/BasicTypes/Sensors/SensorBase.h"

namespace owds {

  Sensor::Sensor(const std::string& id,
                 const std::string& frame_id,
                 bool is_static,
                 const FieldOfView& field_of_view) : Entity(id),
                                                     is_activated_(true),
                                                     is_static_(is_static),
                                                     frame_id_(frame_id),
                                                     world_id_(-1),
                                                     field_of_view_(field_of_view)
  {}

} // namespace owds