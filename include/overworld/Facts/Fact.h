#ifndef OWDS_FACT_H
#define OWDS_FACT_H

#include <string>

namespace owds {

class Fact
{
public:
  Fact(const std::string& subject,
       const std::string& predicate,
       const std::string& object) : subject_(subject),
                                    predicate_(predicate),
                                    object_(object) {}

  std::string getSubject() const { return subject_; }
  std::string getPredicate() const { return predicate_; }
  std::string getObject() const { return object_; }

  std::string toString(const std::string& delim = " ")
  {
    return subject_ + delim + predicate_ + delim + object_;
  }

private:
  std::string subject_;
  std::string predicate_;
  std::string object_;
};

} // namespace owds

#endif // OWDS_FACT_H