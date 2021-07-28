#include "overworld/Bullet/PhysicsServers.h"

#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

#include <ros/package.h>

#include "overworld/Utility/ShellDisplay.h"

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

	int visual_id = client->createVisualShapeBox({1,2,1}, {1,0,0,1});
	int collision_id = client->createCollisionShapeBox({1,0.5,1});
	int obj_id = client->createMultiBody(0, collision_id, visual_id, {3,3,0}, {0,0,0,1});

	std::string path_pr2_description = ros::package::getPath("pr2_description");
	owds::ShellDisplay::info(path_pr2_description);
	path_pr2_description = path_pr2_description.substr(0, path_pr2_description.size() - std::string("/pr2_description").size());
	owds::ShellDisplay::info(path_pr2_description);
	std::string path_overworld = ros::package::getPath("overworld");
	
	client->setAdditionalSearchPath(path_overworld + "/models");
	client->loadURDF("pr2.urdf", {0,0,0}, {0,0,0,1});

	while(flag == false)
	{
		std::this_thread::sleep_for(100ms);
	}

	delete client;

	return 0;
}
