#include "overworld/Bullet/PhysicsServers.h"
#include "overworld/Geometry/Pose.h"
#include "overworld/BasicTypes/Object.h"

#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

#include <ros/package.h>

#include "overworld/Utility/ShellDisplay.h"

#include "overworld/Perception/PR2JointsPerception.h"
#include "overworld/Perception/EntitiesPerceptionManager.h"

using namespace std::chrono_literals;

static volatile std::sig_atomic_t flag;

void sigHandler(int signal)
{
	if(signal == SIGINT)
		flag = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener");
    ros::NodeHandle n;
	flag = false;

	owds::BulletClient* client = owds::PhysicsServers::connectPhysicsServer(owds::CONNECT_GUI);
	std::cout << "server_id " << client->getId() << std::endl;

	int visual_id = client->createVisualShapeBox({1,1.5,1.5}, {1,0,0,1});
	int collision_id = client->createCollisionShapeBox({1,0.5,1});
	int obj_id = client->createMultiBody(0, collision_id, visual_id, {3,3,0}, {0,0,0,1});

	std::string path_pr2_description = ros::package::getPath("pr2_description");
	owds::ShellDisplay::info(path_pr2_description);
	path_pr2_description = path_pr2_description.substr(0, path_pr2_description.size() - std::string("/pr2_description").size());
	owds::ShellDisplay::info(path_pr2_description);
	std::string path_overworld = ros::package::getPath("overworld");
	
	client->setAdditionalSearchPath(path_overworld + "/models");
	int robot_id = client->loadURDF("pr2.urdf", {-2,-2,0}, {0,0,0,1});

	std::cout << "Robot id : " << robot_id << std::endl;
	std::cout << "Nombre de joints: " << client->getNumJoints(robot_id) << std::endl;

	client->configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW, false);
	client->configureDebugVisualizer(COV_ENABLE_SHADOWS, false);

	auto proj_matrix = client->computeProjectionMatrix(60, 1, 0.1, 10);
	auto view_matrix = client->computeViewMatrix({0,0,1}, {-1,-1,1}, {0,0,1});
	auto images = client->getCameraImage(400, 400, view_matrix, proj_matrix, owds::BULLET_HARDWARE_OPENGL);

	std::cout << images.m_pixelWidth << "x" << images.m_pixelHeight << std::endl;

	client->performCollisionDetection();
	auto raycast_info = client->rayTestBatch({{0,0,1}}, {{3,3,1}});
	client->addUserDebugLine({0,0,1}, {3,3,1}, {0,0,1});
	if(raycast_info.m_numRayHits)
	{
		owds::ShellDisplay::info("hit " + std::to_string(raycast_info.m_numRayHits));
		for(size_t i = 0; i < raycast_info.m_numRayHits; i++)
			std::cout << "- " << raycast_info.m_rayHits[i].m_hitObjectUniqueId << " : " << raycast_info.m_rayHits[i].m_hitObjectLinkIndex << std::endl;
	}

	owds::Pose p1;
	owds::Pose p2({{1.0, 1.0, 1.0}}, {{0.0, 0.0, 0.0, 1.0}});
	std::cout << "Distance is: " << p1.distanceTo(p2) << std::endl;

	owds::Object obj1("obj_1"), obj2("table");
	obj1.updatePose({{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0, 1.0}}, ros::Time(1.0));
	obj2.updatePose({{3.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0, 1.0}}, ros::Time(3.0));
	std::cout << "The distance between " << obj1.id() << " and " << obj2.id() << " is: " << obj1.pose().distanceTo(obj2.pose()) << "m." << std::endl;

	owds::PR2JointsPerception joint_perception(&n, robot_id, client);
	owds::EntitiesPerceptionManager<owds::Entity> entities_manager;
	entities_manager.addPerceptionModule("PR2_joints", &joint_perception);

	while(flag == false && ros::ok())
	{
		std::this_thread::sleep_for(100ms);
		ros::spinOnce();
	}

	delete client;

	return 0;
}
