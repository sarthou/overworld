#ifndef OWDS_ONTOLOGENIUSFACTSPUBLISHER_H
#define OWDS_ONTOLOGENIUSFACTSPUBLISHER_H

#include "overworld/Facts/Publisher/FactsPublisher.h"

#include "ontologenius/OntologiesManipulator.h"

namespace owds {

class OntologeniusFactsPublisher : public FactsPublisher
{
public:
  OntologeniusFactsPublisher(ros::NodeHandle* n, const std::string& agent_name);

private:
  std::string agent_name_;
  OntologiesManipulator ontologies_manipulator_;
  OntologyManipulator* onto_;

  void addToKb(const Fact& fact) override;
  void removeFromKb(const Fact& fact) override;
};

} // namespace owds

#endif // OWDS_ONTOLOGENIUSFACTSPUBLISHER_H