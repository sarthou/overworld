#ifndef OWDS_PERCEPT_H
#define OWDS_PERCEPT_H

namespace owds {

template<typename T>
class Percept : public T
{
public:
  template<typename... Args>
  explicit Percept(Args&&... args) : T(std::forward<Args>(args)...)
  {}

private:
  // sensors infos
  // confidence
};

} // namespace owds

#endif // OWDS_PERCEPT_H
