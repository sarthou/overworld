#ifndef OWDS_BULLETCLIENT_H
#define OWDS_BULLETCLIENT_H

#include "SharedMemory/PhysicsClientC_API.h"

#include <string>

namespace owds {

class BulletClient
{
public:
    BulletClient(b3PhysicsClientHandle* client_handle, size_t client_id);
    ~BulletClient();

    size_t getId() { return client_id_; }
    
private:
    b3PhysicsClientHandle* client_handle_;
    size_t client_id_;
};

} // namespace owds

#endif // OWDS_BULLETCLIENT_H