#ifndef OWDS_BODYPART_H
#define OWDS_BODYPART_H

#include "overworld/BasicTypes/Entity.h"

namespace owds {

class BodyPart: public Entity
{
public:
    BodyPart(const std::string& id, bool is_true_id = true);

    std::string getFrameName() const { return frame_name_; }
    void setFrameName(const std::string& frame_name) { frame_name_ = frame_name; }

    std::string getAgentName() const { return agent_name_; }
    void setAgentName(const std::string& name) { agent_name_ = name; }
    bool isAgentKnown() const { return (agent_name_ != ""); }

private:
    std::string frame_name_;
    std::string agent_name_;
};

} // namespace owds

#endif // OWDS_BODYPART_H
