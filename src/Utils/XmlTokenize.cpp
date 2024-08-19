#include "overworld/Utils/XmlTokenize.h"

namespace owds {

  glm::vec3 getVector3FromXmlText(const char* text)
  {
    glm::vec3 vec(0, 0, 0);
    std::vector<float> float_array;
    TokenFloatArray_t adder(float_array);
    float_array.reserve(3);
    tokenize(text, adder);
    assert(float_array.size() == 3);

    vec = glm::vec3(float_array[0], float_array[1], float_array[2]);

    return vec;
  }

  glm::vec4 getVector4FromXmlText(const char* text)
  {
    glm::vec4 vec(0, 0, 0, 0);
    std::vector<float> float_array;
    TokenFloatArray_t adder(float_array);
    float_array.reserve(4);
    tokenize(text, adder);
    assert(float_array.size() == 4);

    vec = glm::vec4(float_array[0], float_array[1], float_array[2], float_array[3]);

    return vec;
  }

} // namespace owds