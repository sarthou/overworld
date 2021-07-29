#ifndef OWDS_BODYPART_H
#define OWDS_BODYPART_H

#include "overworld/BasicTypes/Entity.h"

namespace owds {

class BodyPart: public Entity
{
public:
    BodyPart(const std::string& id);
};

} // namespace owds

#endif // OWDS_BODYPART_H
