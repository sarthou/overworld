#ifndef OWDS_AREAPERCEPTIONMANAGER_H
#define OWDS_AREAPERCEPTIONMANAGER_H

#include <map>
#include <string>

#include "overworld/BasicTypes/Area.h"
#include "overworld/BasicTypes/Object.h"
#include "overworld/BasicTypes/BodyPart.h"

#include "overworld/Bullet/BulletClient.h"
#include "overworld/Perception/Managers/BasePerceptionManager.h"
#include "overworld/Perception/Managers/EntitiesPerceptionManager.h"

#include "overworld/Utility/ShellDisplay.h"

namespace owds {

class AreasPerceptionManager : public BasePerceptionManager<Area>
{
public:
  explicit AreasPerceptionManager(ros::NodeHandle* nh): bullet_client_(nullptr) {}
  ~AreasPerceptionManager();
  
  void setBulletClient(BulletClient* client) { bullet_client_ = client; }

  void registerObjectsManager(EntitiesPerceptionManager<Object>* manager) { objects_managers_.insert(manager); }
  void registerBodyPartsManager(EntitiesPerceptionManager<BodyPart>* manager) { bodyparts_managers_.insert(manager); }

  const std::map<std::string, Area*>& getEntities() const { return areas_; }

  bool update();

private:
  std::map<std::string, Area*> areas_;
  std::map<std::string, Area*> pending_percepts_;
  BulletClient* bullet_client_;

  std::set<EntitiesPerceptionManager<Object>*> objects_managers_;
  std::set<EntitiesPerceptionManager<BodyPart>*> bodyparts_managers_;

  void getPercepts(std::map<std::string, Area>& percepts);

  void solvePendingAreas();

  void addToBullet(Area* area);
  void addPolygonToBullet(Area* area);
  void addCircleToBullet(Area* area);

  Entity* findAreaOwner(Area* area);
};

} // namespace owds

#endif // OWDS_AREAPERCEPTIONMANAGER_H