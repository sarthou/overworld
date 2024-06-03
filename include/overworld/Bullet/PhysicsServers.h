#ifndef OWDS_PHYSICSSERVERS_H
#define OWDS_PHYSICSSERVERS_H

#include <string>

#include "SharedMemory/PhysicsClientC_API.h"
#include "SharedMemory/PhysicsDirectC_API.h"
#include "SharedMemory/SharedMemoryInProcessPhysicsC_API.h"
#include "overworld/Bullet/BulletClient.h"

#define MAX_PHYSICS_CLIENTS 1024

namespace owds {

  enum BulletConnectionMothod_e
  {
    UNCONNECTED,
    CONNECT_GUI,
    CONNECT_GUI_MAIN_THREAD,
    CONNECT_GUI_SERVER,
    CONNECT_GRAPHICS_SERVER_MAIN_THREAD,
    CONNECT_GRAPHICS_SERVER,
    CONNECT_SHARED_MEMORY_SERVER,
    CONNECT_SHARED_MEMORY_GUI,
    CONNECT_DIRECT,
    CONNECT_SHARED_MEMORY
  };

  class PhysicsServers
  {
  public:
    static BulletClient* connectPhysicsServer(BulletConnectionMothod_e methode, size_t port = 1234, size_t key = 12347);
    static bool disconnectPhysicsServer(size_t physics_client_id);

  private:
    PhysicsServers() {}

    static int nb_used_clients_;
    static b3PhysicsClientHandle clients_handles[MAX_PHYSICS_CLIENTS];
    static int clients_method[MAX_PHYSICS_CLIENTS];

    static b3PhysicsClientHandle getPhysicsClient(int physics_client_id);
  };

} // namespace owds

#endif // OWDS_PHYSICSSERVERS_H