#ifndef OVERWORLD_REASONEREGOCENTRIC_H
#define OVERWORLD_REASONEREGOCENTRIC_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

#include "overworld/GetRelations.h"

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
  ros::NodeHandle n_;
  ros::ServiceClient overworld_client_;

  ObjectPropertyBranch_t* isComputableProperty(const std::string& property);
  std::set<ObjectPropertyBranch_t*> isInRange(const std::string& indiv, ObjectPropertyBranch_t* property);
  std::set<ObjectPropertyBranch_t*> isInDomain(const std::string& indiv, ObjectPropertyBranch_t* property);

  overworld::GetRelations createRequest(const std::string& subject, const std::set<ObjectPropertyBranch_t*>& predicates, const std::string& object);
  bool call(overworld::GetRelations& srv);
  void updateOntology(const std::vector<overworld::Triplet>& to_add, const std::vector<overworld::Triplet>& to_delete);
};

} // namespace ontologenius

#endif // OVERWORLD_REASONEREGOCENTRIC_H
