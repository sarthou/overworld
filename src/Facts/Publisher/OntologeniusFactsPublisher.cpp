#include "overworld/Facts/Publisher/OntologeniusFactsPublisher.h"

namespace owds {

OntologeniusFactsPublisher::OntologeniusFactsPublisher(ros::NodeHandle* n,
                                                       const std::string& agent_name) : agent_name_(agent_name),
                                                                                        ontologies_manipulator_(n)
{
  ontologies_manipulator_.waitInit();
  ontologies_manipulator_.add("robot");
  onto_ = ontologies_manipulator_.get("robot");
  onto_->close();
}

void OntologeniusFactsPublisher::addToKb(const Fact& fact)
{
  std::cout << "[ADD] " << agent_name_ << " : " << fact.toString(" - ") << std::endl;
  onto_->feeder.addProperty(fact.getSubject(), fact.getPredicate(), fact.getObject());
}

void OntologeniusFactsPublisher::removeFromKb(const Fact& fact)
{
  std::cout << "[DELETE] " << agent_name_ << " : " << fact.toString(" - ") << std::endl;
  onto_->feeder.removeProperty(fact.getSubject(), fact.getPredicate(), fact.getObject());
}
  
}