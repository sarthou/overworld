#ifndef OWDS_KEYPRESSHANDLER_H
#define OWDS_KEYPRESSHANDLER_H

#include "overworld/Engine/Engine.h"
#include "overworld/Geometry/GeometryUtils.h"
#include "overworld/Perception/PerceptionManagers.h"
#include "overworld/Utils/ShellDisplay.h"
#include "overworld/Engine/Engine.h"

namespace owds {

  inline void onEnterPressed(Engine* engine, const RobotsPerceptionManager& robots_manager)
  {
    auto agents = robots_manager.getAgents();
    if(agents.size() != 1)
      return;

    Agent* agent = agents.begin()->second;
    if(agent != nullptr && agent->getHead() != nullptr && agent->getHead()->isLocated())
    {
      auto head_pose = agent->getHead()->pose();
      auto head_pose_arrays = head_pose.arrays();

      Pose cam_pose_in_head({0., 1., -3.}, {0., 0., 0., 1.});
      Pose cam_in_world = head_pose * cam_pose_in_head;
      auto cam_pose_arrays = cam_in_world.arrays();

      engine->setVizualizerCamera(cam_pose_arrays.first, head_pose_arrays.first);
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

  inline void handleKeypress(Key_e key, bool pressed, Engine* engine, PerceptionManagers& managers)
  {
      if(pressed)
      {
        switch(key)
        {
        case Key_e::key_enter:
          onEnterPressed(engine, managers.robots_manager_);
          break;
        case key_a:
          onAPressed(managers.areas_manager_);
          break;
        default:
          break;
        }
      }
  }

} // namespace owds

#endif /* OWDS_KEYPRESSHANDLER_H */
