#include "overworld/Bullet/PhysicsServers.h"
#include "overworld/Geometry/Pose.h"
#include "overworld/BasicTypes/Object.h"

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
	owds::PhysicsServers servers;

	int server_id = servers.connectPhysicsServer(owds::CONNECT_GUI);
	std::cout << "server_id " << server_id << std::endl;

	owds::Pose p1;
	owds::Pose p2({{1.0, 1.0, 1.0}}, {{0.0, 0.0, 0.0, 1.0}});
	std::cout << "Distance is: " << p1.distanceTo(p2) << std::endl;

	owds::Object obj1("obj_1"), obj2("table");
	obj1.updatePose({{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0, 1.0}}, ros::Time(1.0));
	obj2.updatePose({{3.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0, 1.0}}, ros::Time(3.0));
	std::cout << "The distance between " << obj1.id() << " and " << obj2.id() << " is: " << obj1.pose().distanceTo(obj2.pose()) << "m." << std::endl;

	while(flag == false)
	{
		std::this_thread::sleep_for(100ms);
	}

	return 0;
}
