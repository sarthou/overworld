#ifndef OWDS_BULLETKEYPRESSHANDLER_H
#define OWDS_BULLETKEYPRESSHANDLER_H

#include "overworld/Bullet/BulletClient.h"
#include "overworld/Geometry/GeometryUtils.h"
#include "overworld/Perception/PerceptionManagers.h"
#include "overworld/Utils/ShellDisplay.h"

namespace owds {

  inline void onSpacebarPressed(BulletClient* world_client, const RobotsPerceptionManager& robots_manager)
  {
    auto agents = robots_manager.getAgents();
    if(agents.size() != 1)
      return;

    Agent* agent = agents.begin()->second;
    if(agent != nullptr && agent->getHead() != nullptr &&
       agent->getHead()->isLocated())
    {
      auto head_pose = agent->getHead()->pose();
      auto head_pose_arrays = head_pose.arrays();
      std::array<float, 3> head_posef = {(float)head_pose_arrays.first.at(0), (float)head_pose_arrays.first.at(1),
                                         (float)head_pose_arrays.first.at(2)};
      double yaw = getCameraYawFromHeadPose(head_pose) * 180. / M_PI - 90.; // This yaw angle is defined wrt the map y axis
      world_client->resetDebugVisualizerCamera(3.0, yaw, -30, head_posef);
    }
  }

  inline void onAPressed(AreasPerceptionManager& areas_manager)
  {
    if(areas_manager.isDrawn())
    {
      areas_manager.undrawAreas();
      ShellDisplay::success("Area visualization has been disabled");
    }
    else
    {
      areas_manager.drawAreas();
      ShellDisplay::success("Area visualization has been enabled");
    }
  }

  inline void handleKeypress(BulletClient* world_client, PerceptionManagers& managers)
  {
    b3KeyboardEventsData keyboard = world_client->getKeyboardEvents();
    for(size_t i = 0; i < keyboard.m_numKeyboardEvents; i++)
    {
      if(keyboard.m_keyboardEvents[i].m_keyState & b3VRButtonInfo::eButtonReleased)
      {
        switch(keyboard.m_keyboardEvents[i].m_keyCode)
        {
        case ' ':
          onSpacebarPressed(world_client, managers.robots_manager_);
          break;
        case 'a':
        case 'A':
          onAPressed(managers.areas_manager_);
          break;
        case 65281: // F2
          ShellDisplay::success("Debug visualizer has been disabled");
          world_client->configureDebugVisualizer(COV_ENABLE_RENDERING, false);
          break;
        default:
          // std::cout << keyboard.m_keyboardEvents[i].m_keyCode << std::endl;
          break;
        }
      }
    }
  }

} // namespace owds

#endif /* OWDS_BULLETKEYPRESSHANDLER_H */
