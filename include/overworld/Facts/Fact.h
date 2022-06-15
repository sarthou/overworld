#ifndef OWDS_FACT_H
#define OWDS_FACT_H

#include <string>
#include <functional>

namespace owds {

class Fact
{
public:
  Fact(const std::string& subject,
       const std::string& predicate,
       const std::string& object) : subject_(subject),
                                    predicate_(predicate),
                                    object_(object)
  {
    hash_ = std::hash<std::string>{}(toString());
  }

  const std::string& getSubject() const { return subject_; }
  const std::string& getPredicate() const { return predicate_; }
  const std::string& getObject() const { return object_; }

  size_t getHash() const { return hash_; }
  bool operator==(const Fact& other) const
  {
    return hash_ == other.hash_;
  }

  std::string toString(const std::string& delim = " ") const
  {
    return subject_ + delim + predicate_ + delim + object_;
  }

private:
  std::string subject_;
  std::string predicate_;
  std::string object_;
  size_t hash_;
};

} // namespace owds

#endif // OWDS_FACT_H