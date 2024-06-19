#ifndef OWDS_HELPER_GLMMATH_H
#define OWDS_HELPER_GLMMATH_H

#include "overworld/Helper/BitCast.h"

#define ToV3(x) owds::BitCast<glm::vec3>(x)
#define FromV3(x) owds::BitCast<std::array<float, 3>>(x)
#define ToM4(x) owds::BitCast<glm::mat4>(x)
#define FromM4(x) owds::BitCast<std::array<float, 16>>(x)

#endif // OWDS_HELPER_GLMMATH_H
