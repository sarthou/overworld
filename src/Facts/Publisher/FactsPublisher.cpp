#include "overworld/Facts/Publisher/FactsPublisher.h"

namespace owds {

void FactsPublisher::publish(const std::vector<Fact>& facts)
{
  std::unordered_set<size_t> done_facts;
  std::vector<Fact> to_insert;

  for(auto& fact : facts)
  {
    if(fact.getPredicate() == "pick")
    {
      if(held_by_.find(fact.getObject()) == held_by_.end())
      {
        held_by_.insert(fact.getObject());
        to_insert.push_back(fact);
      }
    }
    else if((fact.getPredicate() == "place") || (fact.getPredicate() == "release"))
    {
      if(held_by_.find(fact.getObject()) != held_by_.end())
      {
        held_by_.erase(fact.getObject());
        removeFromKb(fact);
      }
    }
    else
    {
      if(pending_facts_.find(fact.getHash()) == pending_facts_.end())
      {
        pending_facts_.insert(std::make_pair(fact.getHash(), fact));
        to_insert.push_back(fact);
      }

      done_facts.insert(fact.getHash());
    }
  }

  std::unordered_set<size_t> to_remove;
  for(auto fact_it : pending_facts_)
  {
    if(done_facts.find(fact_it.first) == done_facts.end())
      to_remove.insert(fact_it.first);
  }

  for(auto hash : to_remove)
  {
    removeFromKb(pending_facts_.at(hash));
    pending_facts_.erase(hash);
  }

  for(const auto& fact : to_insert)
    addToKb(fact);
}

} // namespace owds