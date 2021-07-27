#include "overworld/Bullet/BulletClient.h"

#include "overworld/Bullet/PhysicsServers.h"

namespace owds {

BulletClient::BulletClient(b3PhysicsClientHandle* client_handle, size_t client_id)
{
    client_handle_ = client_handle;
    client_id_ = client_id;
}

BulletClient::~BulletClient()
{
    PhysicsServers::disconnectPhysicsServer(client_id_);
}

} // namespace owds