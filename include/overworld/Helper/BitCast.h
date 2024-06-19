#ifndef OWDS_HELPER_BITCAST_H
#define OWDS_HELPER_BITCAST_H

#include <cstring> // std::memcpy
#include <memory>  // std::addressof

namespace owds {
  /**
   * An implementation of C++20's std::bit_cast
   */
  template<class T2, class T1>
  T2 BitCast(T1 t1)
  {
    static_assert(sizeof(T1) == sizeof(T2), "Types must match sizes");
    static_assert(std::is_pod_v<T1>, "Requires POD input");
    static_assert(std::is_pod_v<T2>, "Requires POD output");

    T2 t2;
    std::memcpy(std::addressof(t2), std::addressof(t1), sizeof(T1));
    return t2;
  }
} // namespace owds

#endif // OWDS_HELPER_BITCAST_H