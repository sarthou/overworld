#include "overworld/Bullet/PhysicsServers.h"
#include "overworld/Geometry/Pose.h"
#include "overworld/BasicTypes/Object.h"
#include "overworld/BasicTypes/FieldOfView.h"
#include "overworld/Senders/ROSSender.h"

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

void createCube(owds::BulletClient* client, const std::array<double, 3>& pose, const std::array<double, 4>& color)
{
  int visual_id = client->createVisualShapeBox({0.5,0.5,0.5}, color);
	int collision_id = client->createCollisionShapeBox({0.5,0.5,0.5});
	int obj_id = client->createMultiBody(0, collision_id, visual_id, pose, {0,0,0,1});
}

void createScreen(owds::BulletClient* client, double height, double width)
{
  double dist = 6;
  auto w = width*TO_HALF_RAD;
  auto h = height*TO_HALF_RAD;
  auto left = dist*std::tan(w);
  auto top = dist*std::tan(h);

  int visual_top_id = client->createVisualShapeBox({0.05,left,0.05}, {0,0,0,1});
	int collision_top_id = client->createCollisionShapeBox({0.05,left,0.05});
	int obj_top_id = client->createMultiBody(0, collision_top_id, visual_top_id, {dist,0,top}, {0,0,0,1});

  int visual_bot_id = client->createVisualShapeBox({0.05,left,0.05}, {0,0,0,1});
	int collision_bot_id = client->createCollisionShapeBox({0.05,left,0.05});
	int obj_bot_id = client->createMultiBody(0, collision_bot_id, visual_bot_id, {dist,0,-top}, {0,0,0,1});

  int visual_left_id = client->createVisualShapeBox({0.05,0.05,top}, {0,0,0,1});
	int collision_left_id = client->createCollisionShapeBox({0.05,0.05, top});
	int obj_left_id = client->createMultiBody(0, collision_left_id, visual_left_id, {dist,left,0}, {0,0,0,1});

  int visual_right_id = client->createVisualShapeBox({0.05,0.05,top}, {0,0,0,1});
	int collision_right_id = client->createCollisionShapeBox({0.05,0.05,top});
	int obj_right_id = client->createMultiBody(0, collision_right_id, visual_right_id, {dist,-left,0}, {0,0,0,1});
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bullet_fov");
  ros::NodeHandle n;
  owds::ROSSender ros_sender(&n);
	flag = false;

	owds::BulletClient* client = owds::PhysicsServers::connectPhysicsServer(owds::CONNECT_GUI);

	client->configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW, false);
	client->configureDebugVisualizer(COV_ENABLE_SHADOWS, false);

  createCube(client, {5,0,0}, {1,0,0,1});
  createCube(client, {5,-2,0}, {0,1,0,1});
  createCube(client, {5,2,0}, {0,1,0,1});
  createCube(client, {5,0,-2}, {0,0,1,1});
  createCube(client, {5,0,2}, {0,0,1,1});

  owds::FieldOfView fov(70, 102.4, 0.1, 100);
  owds::FieldOfView fov_1_1(70, 70, 0.1, 100);

  createScreen(client, fov.getHeight(), fov.getWidth());

  owds::Pose head_pose({0,0,0}, {0.5,0.5,0.5,0.5});
  owds::Pose target_pose = head_pose * owds::Pose({0,0,1}, {0,0,0,1});

  double ratio = fov.getRatioOpenGl();
  std::cout << "ratio = " << ratio << std::endl;
  std::cout << "ratio 1x1 = " << fov_1_1.getRatioOpenGl() << std::endl;

    auto proj_matrix = client->computeProjectionMatrix(fov.getHeight(),
                                                      ratio, // fov.getRatio(),
                                                      fov.getClipNear(),
                                                      fov.getClipFar());
    /*auto w = fov.getWidth()*TO_HALF_RAD;
    auto h = fov.getHeight()*TO_HALF_RAD;
    auto left = std::tan(w);
    auto top = std::tan(h);
    std::cout << "w :" << w << "h : " << h << " left : " << left << "top : " << top << std::endl;
    auto proj_matrix = client->computeProjectionMatrix(left,
                                                       -left,
                                                       -top,
                                                       top,
                                                       fov.getClipNear(),
                                                       fov.getClipFar());*/
  auto head_pose_trans = head_pose.arrays().first;
  auto target_pose_trans = target_pose.arrays().first;
  auto view_matrix = client->computeViewMatrix({(float)head_pose_trans[0], (float)head_pose_trans[1], (float)head_pose_trans[2]},
                                                {(float)target_pose_trans[0], (float)target_pose_trans[1], (float)target_pose_trans[2]},
                                                {0.,0.,1.});
  auto images = client->getCameraImage(1000*ratio, 1000, view_matrix, proj_matrix, owds::BULLET_HARDWARE_OPENGL);

	std::cout << "image pixels: " << images.m_pixelWidth << "x" << images.m_pixelHeight << std::endl;

	while(flag == false && ros::ok())
	{
		std::this_thread::sleep_for(100ms);
		ros::spinOnce();
      ros_sender.sendImage("overworld/view", images);
	}

	delete client;

	return 0;
}
