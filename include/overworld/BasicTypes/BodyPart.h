#ifndef OWDS_BODYPART_H
#define OWDS_BODYPART_H

#include "overworld/BasicTypes/Entity.h"

namespace owds {

class BodyPart: public Entity
{
public:
    BodyPart(const std::string& id, bool is_true_id = true);

    std::string getFrameName() { return frame_name_; }
    void setFrameName(const std::string& frame_name) } { frame_name_ = frame_name; }

private:
    std::string frame_name_;
};

} // namespace owds

#endif // OWDS_BODYPART_H
