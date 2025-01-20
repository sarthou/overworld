#ifndef OWDS_SENSORBASE_H
#define OWDS_SENSORBASE_H

#include <unordered_map>
#include <unordered_set>

#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/FieldOfView.h"
#include "overworld/BasicTypes/PointOfInterest.h"
#include "overworld/Utils/CircularBuffer.h"

namespace owds {

  class Sensor : public Entity
  {
  public:
    explicit Sensor(const std::string& id,
                    const std::string& frame_id,
                    bool is_static = false,
                    const FieldOfView& field_of_view = FieldOfView(60, 80, 0.1, 12));

    virtual ~Sensor() = default;

    void clearPoses() { objects_seen_ids_.clear(); }

    void setPerceptseen(const std::string& percept_id) { objects_seen_ids_.insert(percept_id); };
    void resetPerceptseen(const std::string& percept_id) { objects_seen_ids_.erase(percept_id); };

    bool isActivated() const { return is_activated_; }
    void activate(bool is_activated) { is_activated_ = is_activated; }

    const std::string& getFrameId() const { return frame_id_; }
    bool isStatic() const { return is_static_; }
    std::unordered_set<std::string> getObjectsSeen() const { return objects_seen_ids_; }

    const FieldOfView& getFieldOfView() const { return field_of_view_; }

    const std::string& getAgentName() const { return agent_name_; }
    void setAgentName(const std::string& name) { agent_name_ = name; }
    bool isAgentKnown() const { return (agent_name_ != ""); }

    void setWorldSegmentationId(int id) { segmentation_id_ = id; }
    int getWorldSegmentationId() const { return segmentation_id_; }

    void setWorldRgbaId(int id) { rgba_id_ = id; }
    int getWorldRgbaId() const { return rgba_id_; }

  protected:
    bool is_activated_;
    bool is_static_;
    std::string agent_name_;
    std::string frame_id_;

    int segmentation_id_;
    int rgba_id_;

    FieldOfView field_of_view_;

    std::unordered_set<std::string> objects_seen_ids_;
  };

} // namespace owds

#endif // OWDS_AGENT_H