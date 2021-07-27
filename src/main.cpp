#include "overworld/Bullet/PhysicsServers.h"

#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

static volatile std::sig_atomic_t flag;

void sigHandler(int signal)
{
	if(signal == SIGINT)
		flag = true;
}

int main(int argc, char** argv)
{
	flag = false;

	owds::BulletClient* client = owds::PhysicsServers::connectPhysicsServer(owds::CONNECT_GUI);
	std::cout << "server_id " << client->getId() << std::endl;

	while(flag == false)
	{
		std::this_thread::sleep_for(100ms);
	}

	delete client;

	return 0;
}
