#define OWDS_TESTS

#include "overworld/Senders/ApproachSender.h"

int main()
{
  owds::ApproachSender sender(nullptr, nullptr);

  auto node = sender.constraintToTree("a & b | c");
  node.print();
  std::cout << "-----" << std::endl;
  node = sender.constraintToTree("a & (b | c)");
  node.print();
  std::cout << "-----" << std::endl;
  node = sender.constraintToTree("a & (b | c) & !(d | e)");
  node.print();
  std::cout << "-----" << std::endl;
  node = sender.constraintToTree("a & (b | (c & e))");
  node.print();
  std::cout << "-----" << std::endl;
  node = sender.constraintToTree("a&(b|(!c&!e))");
  node.print();
  std::cout << "---END---" << std::endl;

  return 0;
}