#ifndef OWDS_PERCEPT_H
#define OWDS_PERCEPT_H

#include "overworld/BasicTypes/Sensors/SensorBase.h"

namespace owds {

  template<typename T>
  class Percept : public T
  {
  public:
    template<typename... Args>
    explicit Percept(Args&&... args) : T(std::forward<Args>(args)...)
    {}

    std::string getSensorId() const { return sensor_id_; }
    void setSensorId(const std::string& sensor_id) { sensor_id_ = sensor_id; }

    std::string getModuleName() const { return module_name_; }
    void setModuleName(const std::string& module_name) { module_name_ = module_name; }

    float getConfidence() const { return confidence_; }
    void setConfidence(float confidence) { confidence_ = confidence; }

  private:
    std::string sensor_id_;
    std::string module_name_;
    float confidence_;
  };

} // namespace owds

#endif // OWDS_PERCEPT_H