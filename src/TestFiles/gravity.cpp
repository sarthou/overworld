#include "overworld/Bullet/PhysicsServers.h"
#include "overworld/Geometry/Pose.h"

#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

#include <ros/package.h>
#include <ros/ros.h>

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
	ros::init(argc, argv, "listener");
    ros::NodeHandle n;
	flag = false;

	owds::BulletClient* client = owds::PhysicsServers::connectPhysicsServer(owds::CONNECT_GUI);
	std::cout << "server_id " << client->getId() << std::endl;

  std::string meshes_path = ros::package::getPath("overworld") + "/models/test_objects/";

  int ball_visual_id = client->createVisualShapeMesh(meshes_path + "ball.obj", {1,1,1}, {1,0,0,1});
  int ball_collision_id = client->createCollisionShapeMesh(meshes_path + "ball.obj", {1,1,1});
  int ball_obj_id = client->createMultiBody(0, ball_collision_id, ball_visual_id, {0,0,3}, {0,0,0,1});

  int small_box_visual_id = client->createVisualShapeMesh(meshes_path + "small_box.obj", {1,1,1}, {0,1,0,0.5});
  int small_box_collision_id = client->createCollisionShapeMesh(meshes_path + "small_box_vhacd.obj", {1,1,1});
  int small_box_obj_id = client->createMultiBody(0, small_box_collision_id, small_box_visual_id, {0,0,2}, {0,0,0,1});

  int large_box_visual_id = client->createVisualShapeMesh(meshes_path + "large_box.obj", {1,1,1}, {0,0,1,0.5});
  int large_box_collision_id = client->createCollisionShapeMesh(meshes_path + "large_box_vhacd.obj", {1,1,1});
  int large_box_obj_id = client->createMultiBody(0, large_box_collision_id, large_box_visual_id, {0,0,1}, {0,0,0,1});

  meshes_path = ros::package::getPath("overworld") + "/models/adream/";

  int floor_visual_id = client->createVisualShapeMesh(meshes_path + "floor.obj", {1,1,1});
  int floor_collision_id = client->createCollisionShapeMesh(meshes_path + "floor.obj", {1,1,1});
  int floor_obj_id = client->createMultiBody(0, floor_collision_id, floor_visual_id, {-1,0,0}, {0,0,0,1});

  std::cout << "-----" << floor_obj_id << "-----------" << std::endl;

  client->setMass(ball_obj_id, -1, 0.1);
  client->setMass(small_box_obj_id, -1, 0.4);
  client->setMass(large_box_obj_id, -1, 0.5);

  client->setRestitution(ball_obj_id, -1, 0.1);
  client->setRestitution(small_box_obj_id, -1, 0.001);
  client->setRestitution(large_box_obj_id, -1, 0.001);

  client->setFrictionAnchor(ball_obj_id, -1, 1);
  client->setFrictionAnchor(small_box_obj_id, -1, 1);
  client->setFrictionAnchor(large_box_obj_id, -1, 1);

  /*client->setActivationState(ball_obj_id, eActivationStateEnableSleeping);
  client->setActivationState(small_box_obj_id, eActivationStateDisableSleeping);
  client->setActivationState(large_box_obj_id, eActivationStateEnableSleeping);*/

  client->setGravity(0, 0, -9.81);
  client->setTimeStep(0.02);

	while(flag == false && ros::ok())
	{
		std::this_thread::sleep_for(20ms);
		ros::spinOnce();
        client->stepSimulation();
	}

	delete client;

	return 0;
}
