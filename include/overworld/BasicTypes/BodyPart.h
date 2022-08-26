#ifndef OWDS_BODYPART_H
#define OWDS_BODYPART_H

#include "overworld/BasicTypes/Entity.h"

namespace owds {

enum BodyPartType_e
{
    BODY_PART_UNKNOW,
    BODY_PART_HEAD,
    BODY_PART_LEFT_HAND,
    BODY_PART_RIGHT_HAND,
    BODY_PART_TORSO,
    BODY_PART_BASE
};

class BodyPart: public Entity
{
public:
    explicit BodyPart(const std::string& id, bool is_true_id = true);

    const std::string& getFrameName() const { return frame_name_; }
    void setFrameName(const std::string& frame_name) { frame_name_ = frame_name; }

    const std::string& getAgentName() const { return agent_name_; }
    void setAgentName(const std::string& name) { agent_name_ = name; }
    bool isAgentKnown() const { return (agent_name_ != ""); }

    BodyPartType_e getType() const { return type_; }
    void setType(BodyPartType_e type) { type_ = type; }

private:
    std::string frame_name_;
    std::string agent_name_;
    BodyPartType_e type_;
};

} // namespace owds

#endif // OWDS_BODYPART_H
