#include <chrono>
#include <csignal>
#include <iostream>
#include <ros/package.h>
#include <thread>

#include "overworld/Bullet/PhysicsServers.h"
#include "overworld/Utils/ShellDisplay.h"

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

  owds::BulletClient* client_gui = owds::PhysicsServers::connectPhysicsServer(owds::CONNECT_GUI);
  std::cout << "server gui id " << client_gui->getId() << std::endl;

  owds::BulletClient* client_direct = owds::PhysicsServers::connectPhysicsServer(owds::CONNECT_DIRECT);
  std::cout << "server direct id " << client_direct->getId() << std::endl;

  int visual_id = client_gui->createVisualShapeBox({1, 1.5, 1.5}, {1, 0, 0, 1});
  int collision_id = client_gui->createCollisionShapeBox({1, 1.5, 1.5});
  int obj_id = client_gui->createMultiBody(0, collision_id, visual_id, {3, 3, 0}, {0, 0, 0, 1});
  std::cout << "create object gui: " << obj_id << std::endl;

  visual_id = client_direct->createVisualShapeBox({1, 1, 1}, {0, 1, 0, 1});
  collision_id = client_direct->createCollisionShapeBox({1, 1, 1});
  obj_id = client_direct->createMultiBody(0, collision_id, visual_id, {-3, -3, 0}, {0, 0, 0, 1});
  std::cout << "create object direct: " << obj_id << std::endl;

  auto proj_matrix = client_gui->computeProjectionMatrix(60, 1, 0.1, 10);
  auto view_matrix = client_gui->computeViewMatrix({0, 0, 1}, {1, 1, 1}, {0, 0, 1});
  auto images = client_gui->getCameraImage(400, 400, view_matrix, proj_matrix, owds::BULLET_HARDWARE_OPENGL);

  proj_matrix = client_direct->computeProjectionMatrix(60, 1, 0.1, 10);
  view_matrix = client_direct->computeViewMatrix({0, 0, 1}, {-1, -1, 1}, {0, 0, 1});
  images = client_direct->getCameraImage(400, 400, view_matrix, proj_matrix, owds::TINY_RENDERER);

  std::cout << images.m_pixelWidth << "x" << images.m_pixelHeight << std::endl;

  client_gui->performCollisionDetection();
  auto raycast_info = client_gui->rayTestBatch({
                                                 {0, 0, 1}
  },
                                               {{3, 3, 1}});
  client_gui->addUserDebugLine({0, 0, 1}, {3, 3, 1}, {0, 0, 1});
  if(raycast_info.size())
  {
    owds::ShellDisplay::info("hit gui " + std::to_string(raycast_info.size()));
    for(auto& info : raycast_info)
      std::cout << "- " << info.m_hitObjectUniqueId << " : " << info.m_hitObjectLinkIndex << std::endl;
  }

  client_direct->performCollisionDetection();
  raycast_info = client_direct->rayTestBatch({
                                               {0, 0, 1}
  },
                                             {{-3, -3, 0.5}});
  if(raycast_info.size())
  {
    owds::ShellDisplay::info("hit direct " + std::to_string(raycast_info.size()));
    for(auto& info : raycast_info)
      std::cout << "- " << info.m_hitObjectUniqueId << " : " << info.m_hitObjectLinkIndex << std::endl;
  }

  while(flag == false)
  {
    std::this_thread::sleep_for(100ms);
  }

  delete client_gui;

  return 0;
}
