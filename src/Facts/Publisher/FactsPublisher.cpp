#include "overworld/Facts/Publisher/FactsPublisher.h"

#include <algorithm>
#include <iostream>

namespace owds {

void FactsPublisher::publish(const std::vector<Fact>& facts)
{
  std::unordered_set<size_t> done_facts;
  std::vector<Fact> to_insert;

  auto filtered_facts = filterIncomingFacts(facts);

  for(auto& fact : filtered_facts)
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

  auto stable_to_remove = filterOutgoingFacts(to_remove);
  for(auto hash : stable_to_remove)
  {
    removeFromKb(pending_facts_.at(hash));
    pending_facts_.erase(hash);
  }

  for(const auto& fact : to_insert)
    addToKb(fact);
}

std::vector<Fact> FactsPublisher::filterIncomingFacts(const std::vector<Fact>& facts)
{
  auto buf_fact_it = facts_buffer_.begin();
  for(; buf_fact_it != facts_buffer_.end();)
  {
    auto found_it = std::find_if(facts.begin(), facts.end(), [buf_fact_it](const auto& fact) { return fact == buf_fact_it->first; });

    if(found_it == facts.end())
      buf_fact_it = facts_buffer_.erase(buf_fact_it);
    else
      ++buf_fact_it;
  }

  std::vector<Fact> stable_facts;
  for(auto& fact : facts)
  {
    auto buf_it = facts_buffer_.find(fact);
    if(buf_it == facts_buffer_.end())
      facts_buffer_.insert({fact, 0});
    else if(buf_it->second == 3)
      stable_facts.push_back(fact);
    else
      buf_it->second++;
  }

  return stable_facts;
}

std::unordered_set<size_t> FactsPublisher::filterOutgoingFacts(const std::unordered_set<size_t>& facts)
{
  auto buf_fact_it = rmv_buffer_.begin();
  for(; buf_fact_it != rmv_buffer_.end();)
  {
    auto found_it = std::find_if(facts.begin(), facts.end(), [buf_fact_it](const auto& fact) { return fact == buf_fact_it->first; });

    if(found_it == facts.end())
      buf_fact_it = rmv_buffer_.erase(buf_fact_it);
    else
      ++buf_fact_it;
  }

  std::unordered_set<size_t> stable_facts;
  for(auto& fact : facts)
  {
    auto buf_it = rmv_buffer_.find(fact);
    if(buf_it == rmv_buffer_.end())
      rmv_buffer_.insert({fact, 0});
    else if(buf_it->second == 3)
    {
      stable_facts.insert(fact);
      rmv_buffer_.erase(buf_it);
    }
    else
      buf_it->second++;
  }

  return stable_facts;
}

} // namespace owds