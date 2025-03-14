#include "overworld/Facts/Publisher/OntologeniusFactsPublisher.h"

namespace owds {

  OntologeniusFactsPublisher::OntologeniusFactsPublisher(const std::string& agent_name) : agent_name_(agent_name)
  {
    ontologies_manipulator_.waitInit();
    ontologies_manipulator_.add(agent_name_);
    onto_ = ontologies_manipulator_.get(agent_name_);
    onto_->close();
  }

  void OntologeniusFactsPublisher::addToKb(const Fact& fact)
  {
    std::cout << "[ADD] " << agent_name_ << " : " << fact.toString(" - ") << std::endl;
    onto_->feeder.addRelation(fact.getSubject(), fact.getPredicate(), fact.getObject());
  }

  void OntologeniusFactsPublisher::removeFromKb(const Fact& fact)
  {
    std::cout << "[DELETE] " << agent_name_ << " : " << fact.toString(" - ") << std::endl;
    onto_->feeder.removeRelation(fact.getSubject(), fact.getPredicate(), fact.getObject());
  }

} // namespace owds