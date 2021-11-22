#ifndef OWDS_FACTSPUBLISHER_H
#define OWDS_FACTSPUBLISHER_H

#include "overworld/Facts/Fact.h"

#include <map>
#include <vector>
#include <unordered_set>

namespace owds {

class FactsPublisher
{
public:
  FactsPublisher() = default;

  void publish(const std::vector<Fact>& facts);

protected:
  virtual void addToKb(const Fact& fact) = 0;
  virtual void removeFromKb(const Fact& fact) = 0;

private:
  std::map<size_t, Fact> pending_facts_;
  std::unordered_set<std::string> held_by_;
};

} // namespace owds

#endif // OWDS_FACTSPUBLISHER_H