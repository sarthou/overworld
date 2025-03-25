#ifndef OVERWORLD_REASONEREGOCENTRIC_H
#define OVERWORLD_REASONEREGOCENTRIC_H

#include <ros/ros.h>

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"
#include "overworld/GetRelations.h"

namespace overworld {

  class ReasonerEgocentric : public ontologenius::ReasonerInterface
  {
  public:
    ReasonerEgocentric() {}
    virtual ~ReasonerEgocentric() = default;

    virtual void initialize() override;

    virtual bool preReason(const ontologenius::QueryInfo_t& query_info) override;

    virtual bool implementPreReasoning() override { return true; }

    virtual std::string getName() override;
    virtual std::string getDescription() override;

  private:
    std::unordered_set<ontologenius::ObjectPropertyBranch*> computable_properties_;
    ros::NodeHandle n_;
    ros::ServiceClient overworld_client_;

    ontologenius::ObjectPropertyBranch* isComputableProperty(const std::string& property);
    std::set<ontologenius::ObjectPropertyBranch*> isInRange(const std::string& indiv, ontologenius::ObjectPropertyBranch* property);
    std::set<ontologenius::ObjectPropertyBranch*> isInDomain(const std::string& indiv, ontologenius::ObjectPropertyBranch* property);

    overworld::GetRelations createRequest(const std::string& subject, const std::set<ontologenius::ObjectPropertyBranch*>& predicates, const std::string& object);
    bool call(overworld::GetRelations& srv);
    void updateOntology(const std::vector<overworld::Triplet>& to_add, const std::vector<overworld::Triplet>& to_delete);
  };

} // namespace overworld

#endif // OVERWORLD_REASONEREGOCENTRIC_H
