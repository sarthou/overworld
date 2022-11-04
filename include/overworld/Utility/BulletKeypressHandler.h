#ifndef OWDS_BULLETKEYPRESSHANDLER_H
#define OWDS_BULLETKEYPRESSHANDLER_H

#include "overworld/Bullet/BulletClient.h"
#include "overworld/Geometry/GeometryUtils.h"
#include "overworld/Perception/Managers/RobotsPerceptionManager.h"
#include "overworld/Utility/ShellDisplay.h"

namespace owds {

inline void onSpacebarPressed(BulletClient* bullet_client, const RobotsPerceptionManager& robots_manager)
{
    auto agents = robots_manager.getAgents();
    if(agents.size() != 1)
        return;

    Agent* agent = agents.begin()->second;
    if (agent != nullptr && agent->getHead() != nullptr &&
        agent->getHead()->isLocated())
    {
        auto head_pose = agent->getHead()->pose();
        auto head_pose_arrays = head_pose.arrays();
        std::array<float, 3> head_posef = {(float)head_pose_arrays.first.at(0), (float)head_pose_arrays.first.at(1),
                                           (float)head_pose_arrays.first.at(2)};
        double yaw = getCameraYawFromHeadPose(head_pose) * 180. / M_PI - 90.;  // This yaw angle is defined wrt the map y axis 
        bullet_client->resetDebugVisualizerCamera(3.0, yaw, -30, head_posef);
    }
}

inline void handleKeypress(BulletClient* bullet_client, const RobotsPerceptionManager& robots_manager)
{
    b3KeyboardEventsData keyboard = bullet_client->getKeyboardEvents();
    for (size_t i = 0; i < keyboard.m_numKeyboardEvents; i++)
    {
        if (keyboard.m_keyboardEvents[i].m_keyState & b3VRButtonInfo::eButtonReleased)
        {
            switch (keyboard.m_keyboardEvents[i].m_keyCode)
            {
            case ' ':
                onSpacebarPressed(bullet_client, robots_manager);
                break;
            case 65281:  // F2
                ShellDisplay::success("Debug visualizer has been disabled");
                bullet_client->configureDebugVisualizer(COV_ENABLE_RENDERING, false);
                break;
            default:
                //std::cout << keyboard.m_keyboardEvents[i].m_keyCode << std::endl;
                break;
            }
        }
    }
}
} // namespace owds
#endif /* OWDS_BULLETKEYPRESSHANDLER_H */
