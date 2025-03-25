#ifndef OWDS_FIRSTMETHODDATAFUSION_H
#define OWDS_FIRSTMETHODDATAFUSION_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>

#include "overworld/Perception/DataFusion/DataFusionBase.h"

namespace owds {

  template<typename T>
  class FirstMethodDataFusion : public DataFusionBase_<T>
  {
  public:
  private:
  };

} // namespace owds

#endif // OWDS_FIRSTMETHODDATAFUSION_H