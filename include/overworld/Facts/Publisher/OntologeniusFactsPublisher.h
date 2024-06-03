#ifndef OWDS_ONTOLOGENIUSFACTSPUBLISHER_H
#define OWDS_ONTOLOGENIUSFACTSPUBLISHER_H

#include "ontologenius/OntologiesManipulator.h"
#include "overworld/Facts/Publisher/FactsPublisher.h"

namespace owds {

  class OntologeniusFactsPublisher : public FactsPublisher
  {
  public:
    OntologeniusFactsPublisher(const std::string& agent_name);

  private:
    std::string agent_name_;
    onto::OntologiesManipulator ontologies_manipulator_;
    onto::OntologyManipulator* onto_;

    void addToKb(const Fact& fact) override;
    void removeFromKb(const Fact& fact) override;
  };

} // namespace owds

#endif // OWDS_ONTOLOGENIUSFACTSPUBLISHER_H