#ifndef OWDS_APPROACHSENDER_H
#define OWDS_APPROACHSENDER_H

#include "ontologenius/OntologyManipulator.h"

#include "overworld/Perception/PerceptionManagers.h"
#include "overworld/BasicTypes/Agent.h"

namespace owds {

enum LogicalAlgebraOperator_e
{
  logical_and,
  logical_or,
  logical_not,
  logical_none
};

class LogicalAlgebraNode
{
public:
  explicit LogicalAlgebraNode(LogicalAlgebraOperator_e op) : operator_(op) {}

  void insert(const std::string& value) { values_.push_back(value); }
  void insert(const LogicalAlgebraNode& node) { nodes_.push_back(node); }

  bool evaluate(const std::string& entity, onto::IndividualClient* client);

  void print(size_t level = 0);

private:
  LogicalAlgebraOperator_e operator_;
  std::vector<std::string> values_;
  std::vector<LogicalAlgebraNode> nodes_;

  bool evaluate(const std::string& entity, const std::string& value, onto::IndividualClient* client);
};

class ApproachSender
{
public:
  ApproachSender(ros::NodeHandle* n, PerceptionManagers* managers);

  void setRobotName(const std::string& robot_name);

#ifndef OWDS_TESTS
private:
#endif
  ros::NodeHandle* n_;
  std::string robot_name_;
  Agent* robot_;

  PerceptionManagers* managers_;
  onto::OntologyManipulator* onto_;

  ros::ServiceServer get_pose_service_;
  
  LogicalAlgebraNode constraintToTree(std::string constraint);
  LogicalAlgebraNode createAndNode(std::string constraint, const std::unordered_map<std::string, LogicalAlgebraNode>& braquet_nodes);
  void fillNode(LogicalAlgebraNode& node, std::string constraint, const std::unordered_map<std::string, LogicalAlgebraNode>& braquet_nodes);
  size_t getIn(size_t begin, std::string& in_text, const std::string& text, char symbol_in, char symbol_out);
  std::vector<std::string> split(const std::string& text, const std::string& delim);
};

} // namespace owds

#endif // OWDS_APPROACHSENDER_H