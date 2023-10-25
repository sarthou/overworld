#include "overworld/Senders/ApproachSender.h"

#include "ontologenius/OntologiesManipulator.h"

namespace owds {

bool LogicalAlgebraNode::evaluate(const std::string& entity, onto::IndividualClient* client)
{
  switch (operator_)
  {
  case logical_and:
    for(auto& value : values_)
      if(!evaluate(entity, value, client))
        return false;
    for(auto& node : nodes_)
      if(!node.evaluate(entity, client))
        return false;
    return true;
  case logical_or:
    for(auto& value : values_)
      if(evaluate(entity, value, client))
        return true;
    for(auto& node : nodes_)
      if(node.evaluate(entity, client))
        return true;
    return false;
  case logical_not:
    if(values_.size())
      return !evaluate(entity, values_.front(), client);
    else if(nodes_.size())
      return !nodes_.front().evaluate(entity, client);
    else
      return false;
  default:
    return false;
  }
}

bool LogicalAlgebraNode::evaluate(const std::string& entity, const std::string& value, onto::IndividualClient* client)
{
  auto res = client->getOn(entity, "isInArea");
  return (std::find(res.begin(), res.end(), value) != res.end());
}

ApproachSender::ApproachSender(ros::NodeHandle* n, PerceptionManagers* managers) : n_(n),
                                                                                   robot_(nullptr),
                                                                                   managers_(managers),
                                                                                   onto_(nullptr)
{}

void ApproachSender::setRobotName(const std::string& robot_name)
{
  robot_name_ = robot_name;
  auto ontos = onto::OntologiesManipulator(n_);
  ontos.waitInit();
  ontos.add(robot_name_);
  onto_ = ontos.get(robot_name_);
  onto_->close();

  robot_ = managers_->robots_manager_.getAgent(robot_name_);
}

} // namespace owds