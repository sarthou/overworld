#ifndef OWDS_FACTSPUBLISHER_H
#define OWDS_FACTSPUBLISHER_H

#include "overworld/Facts/Fact.h"

#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <unordered_map>

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
  std::unordered_map<size_t, Fact> pending_facts_;
  std::unordered_set<std::string> held_by_;

  std::unordered_map<Fact, int> facts_buffer_;
  std::unordered_map<size_t, int> rmv_buffer_;

  std::vector<Fact> filterIncomingFacts(const std::vector<Fact>& facts);
  std::unordered_set<size_t> filterOutgoingFacts(const std::unordered_set<size_t>& facts);
};

} // namespace owds

#endif // OWDS_FACTSPUBLISHER_H