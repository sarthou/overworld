#include "overworld/Senders/ApproachSender.h"

int main()
{
    owds::ApproachSender sender(nullptr, nullptr);

    sender.constraintToTree("a & b | c");
    std::cout << "-----" << std::endl;
    sender.constraintToTree("a & (b | c)");
    std::cout << "-----" << std::endl;
    sender.constraintToTree("a & (b | c) & !(d | e)");
    std::cout << "-----" << std::endl;
    sender.constraintToTree("a & (b | (c & e))");
    std::cout << "-----" << std::endl;
    sender.constraintToTree("a&(b|(c&e))");
    std::cout << "-----" << std::endl;

    return 0;
}