#ifndef BODYPART_H
#define BODYPART_H

#include "overworld/BasicTypes/Entity.h"

namespace owds{

class BodyPart: public Entity{
    public:
    BodyPart();
    BodyPart(const std::string& id);

};
}

#endif /* BODYPART_H */
