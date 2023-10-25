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

LogicalAlgebraNode ApproachSender::constraintToTree(std::string constraint)
{
  std::cout << "in = " << constraint << std::endl;

  std::unordered_map<std::string, LogicalAlgebraNode> braquet_nodes;
  size_t braquet_cpt = 0;
  size_t braquet_pose = constraint.find("(");
  while(braquet_pose != std::string::npos)
  {
    std::string in_braquet;
    size_t out_pose = getIn(braquet_pose, in_braquet, constraint, '(', ')');
    constraint.replace(braquet_pose, out_pose - braquet_pose + 1, "$" + std::to_string(braquet_cpt));
    braquet_nodes.insert(std::make_pair<std::string,LogicalAlgebraNode>("$" + std::to_string(braquet_cpt),
                                                                        constraintToTree(in_braquet)));
    braquet_cpt++;
    braquet_pose = constraint.find("(");
  }

  std::cout << "out = " << constraint << std::endl;

  return LogicalAlgebraNode(logical_none);
}

size_t ApproachSender::getIn(size_t begin, std::string& in_text, const std::string& text, char symbol_in, char symbol_out)
{
  size_t pose = begin;

  if(text.at(pose) == symbol_in)
  {
    size_t first_pose = pose;
    int cpt = 1;
    while((cpt != 0) && (pose+1 < text.length()))
    {
      ++pose;
      if(text.at(pose) == symbol_in)
        cpt++;
      else if(text.at(pose) == symbol_out)
        cpt--;

    }

    in_text = text.substr(first_pose+1, pose-first_pose-1);

    if(cpt == 0)
      return pose;
    else
      return std::string::npos;
  }
  else
    return begin;
}

} // namespace owds