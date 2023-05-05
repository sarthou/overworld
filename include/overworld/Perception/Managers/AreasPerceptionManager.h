#ifndef OWDS_AREAPERCEPTIONMANAGER_H
#define OWDS_AREAPERCEPTIONMANAGER_H

#include <map>
#include <string>

#include "overworld/BasicTypes/Area.h"
#include "overworld/Bullet/BulletClient.h"
#include "overworld/Perception/Managers/BasePerceptionManager.h"

#include "overworld/Utility/ShellDisplay.h"

namespace owds {

class AreasPerceptionManager : public BasePerceptionManager<Area>
{
public:
  explicit AreasPerceptionManager(ros::NodeHandle* nh): bullet_client_(nullptr) {}
  ~AreasPerceptionManager();
  
  void setBulletClient(BulletClient* client) { bullet_client_ = client; }

  const std::map<std::string, Area*>& getEntities() const { return areas_; }

  bool update();

private:
  std::map<std::string, Area*> areas_;
  BulletClient* bullet_client_;

  void getPercepts(std::map<std::string, Area>& percepts);

  void addToBullet(Area* area);
  void addPolygonToBullet(Area* area);
  void addCircleToBullet(Area* area);
};

} // namespace owds

#endif // OWDS_AREAPERCEPTIONMANAGER_H