#ifndef OVERWORLD_REASONEREGOCENTRIC_H
#define OVERWORLD_REASONEREGOCENTRIC_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

namespace ontologenius {

class ReasonerEgocentric : public ReasonerInterface
{
public:
  ReasonerEgocentric() {}
  virtual ~ReasonerEgocentric() = default;

  virtual void initialize() override;

  virtual void preReason(const QueryInfo_t& query_info) override;

  virtual bool implementPreReasoning() override { return true; }

  virtual std::string getName() override;
  virtual std::string getDesciption() override;
private:
  std::unordered_set<ObjectPropertyBranch_t*> computable_properties_;

  ObjectPropertyBranch_t* isComputableProperty(const std::string& property);
  bool isInRange(const std::string& indiv, ObjectPropertyBranch_t* property);
  bool isInDomain(const std::string& indiv, ObjectPropertyBranch_t* property);
};

} // namespace ontologenius

#endif // OVERWORLD_REASONEREGOCENTRIC_H
