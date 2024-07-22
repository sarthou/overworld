#include "overworld/BasicTypes/Sensors/SensorBase.h"

namespace owds {

  Sensor::Sensor(const std::string& id,
                 const std::string& frame_id,
                 bool is_static,
                 const FieldOfView& field_of_view) : Entity(id),
                                                     is_activated_(true),
                                                     is_static_(is_static),
                                                     frame_id_(frame_id),
                                                     field_of_view_(field_of_view)
  {}

  // TODO add measures and real value to calculte the following
  double calculateMeanRelativeError()
  {
    return 0;
    /*
    if(measuredData.empty())
    {
      return 0.0;
    }
    double totalRelativeError = 0.0;
    for(size_t i = 0; i < measuredData.size(); ++i)
    {
      double absoluteError = std::abs(measuredData[i] - realData[i]);
      double relativeError = (absoluteError / realData[i]) * 100.0;
      totalRelativeError += relativeError;
    }
    return totalRelativeError / measuredData.size();*/
  }

  double calculateStandardDeviation()
  {
    return 0;
    /*
    if(measuredData.empty())
    {
      return 0.0;
    }
    double mean = std::accumulate(measuredData.begin(), measuredData.end(), 0.0) / measuredData.size();
    double sumOfSquares = std::accumulate(measuredData.begin(), measuredData.end(), 0.0,
                                          [mean](double a, double b) { return a + (b - mean) * (b - mean); });
    return std::sqrt(sumOfSquares / measuredData.size());*/
  }

  /*    std::vector<double> applyMovingAverageFilter(size_t windowSize) const {
          std::vector<double> filteredData;
          if (windowSize > measuredData.size() || windowSize == 0) {
              return filteredData;
          }
          for (size_t i = 0; i <= measuredData.size() - windowSize; ++i) {
              double sum = std::accumulate(measuredData.begin() + i, measuredData.begin() + i + windowSize, 0.0);
              filteredData.push_back(sum / windowSize);
          }
          return filteredData;
      }
  */

} // namespace owds