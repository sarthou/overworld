#include "overworld/Bullet/PhysicsServers.h"

#include <iostream>

namespace owds {

int PhysicsServers::nb_used_clients_ = 0;
b3PhysicsClientHandle PhysicsServers::clients_handles[MAX_PHYSICS_CLIENTS] = {0};
int PhysicsServers::clients_method[MAX_PHYSICS_CLIENTS] = {0};

BulletClient* PhysicsServers::connectPhysicsServer(BulletConnectionMothod_e method, size_t port, size_t key)
{
	int free_index = -1;
	b3PhysicsClientHandle sm = 0;

	if(nb_used_clients_ >= MAX_PHYSICS_CLIENTS)
	{
		std::cout << "Exceeding maximum number of physics connections." << std::endl;
		return nullptr;
	}

	//Only one local in-process GUI connection allowed.
	if (method == CONNECT_GUI)
	{
		for (size_t i = 0; i < MAX_PHYSICS_CLIENTS; i++)
		{
			if ((clients_method[i] == CONNECT_GUI) || (clients_method[i] == CONNECT_GUI_SERVER))
			{
				std::cout << "Only one local in-process GUI/GUI_SERVER connection allowed. Use DIRECT connection mode or start a separate GUI physics server (ExampleBrowser, App_SharedMemoryPhysics_GUI, App_SharedMemoryPhysics_VR) and connect over SHARED_MEMORY, UDP or TCP instead." << std::endl;
				return nullptr;
			}
		}
	}

	int argc = 0;
	char** argv = 0;

	switch (method)
	{
		case CONNECT_GUI:
		{
			sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
			break;
		}
		case CONNECT_GUI_MAIN_THREAD:
		{
			sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
			break;
		}
		case CONNECT_GUI_SERVER:
		{
			sm = b3CreateInProcessPhysicsServerAndConnectSharedMemory(argc, argv);
			break;
		}
		case CONNECT_GRAPHICS_SERVER_MAIN_THREAD:
		{
			sm = b3CreateInProcessGraphicsServerAndConnectMainThreadSharedMemory(port);
			break;
		}
		case CONNECT_GRAPHICS_SERVER:
		{
			sm = b3CreateInProcessGraphicsServerAndConnectSharedMemory(port);
			break;
		}
		case CONNECT_SHARED_MEMORY_SERVER:
		{
			sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect3(0, key);
			break;
		}
		case CONNECT_SHARED_MEMORY_GUI:
		{
			sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect4(0, key);
			break;
		}
		case CONNECT_DIRECT:
		{
			sm = b3ConnectPhysicsDirect();
			break;
		}
		case CONNECT_SHARED_MEMORY:
		{
			sm = b3ConnectSharedMemory(key);
			break;
		}
		default:
		{
			std::cout << "connectPhysicsServer unexpected argument" << std::endl;
			return nullptr;
		}
	};

	if (sm)
	{
		if (b3CanSubmitCommand(sm))
		{
			for (size_t i = 0; i < MAX_PHYSICS_CLIENTS; i++)
			{
				if (clients_handles[i] == 0)
				{
					free_index = i;
					break;
				}
			}

			if (free_index >= 0)
			{
				b3SharedMemoryCommandHandle command;
				b3SharedMemoryStatusHandle status_handle;
				int status_type;

				clients_handles[free_index] = sm;
				clients_method[free_index] = method;
				nb_used_clients_++;

				if (method != CONNECT_GRAPHICS_SERVER && method != CONNECT_GRAPHICS_SERVER_MAIN_THREAD)
				{
					command = b3InitSyncBodyInfoCommand(sm);
					status_handle = b3SubmitClientCommandAndWaitStatus(sm, command);
					status_type = b3GetStatusType(status_handle);

					if (status_type != CMD_SYNC_BODY_INFO_COMPLETED)
					{
						std::cout << "Connection terminated, couldn't get body info" << std::endl;
						b3DisconnectSharedMemory(sm);
						sm = 0;
						clients_handles[free_index] = 0;
						clients_method[free_index] = 0;
						nb_used_clients_++;
						return nullptr;
					}

					command = b3InitSyncUserDataCommand(sm);
					status_handle = b3SubmitClientCommandAndWaitStatus(sm, command);
					status_type = b3GetStatusType(status_handle);

					if (status_type != CMD_SYNC_USER_DATA_COMPLETED)
					{
						std::cout << "Connection terminated, couldn't get user data" << std::endl;
						b3DisconnectSharedMemory(sm);
						sm = 0;
						clients_handles[free_index] = 0;
						clients_method[free_index] = 0;
						nb_used_clients_++;
						return nullptr;
					}
				}
			}
		}
		else
		{
			b3DisconnectSharedMemory(sm);
		}
	}
	return new BulletClient(&clients_handles[free_index], free_index);
}

bool PhysicsServers::disconnectPhysicsServer(size_t physics_client_id)
{
	b3PhysicsClientHandle sm = getPhysicsClient(physics_client_id);
	if (sm == 0)
	{
		std::cout << "Not connected to physics server." << std::endl;
		return false;
	}

	b3DisconnectSharedMemory(sm);
	sm = 0;

	clients_handles[physics_client_id] = 0;
	clients_method[physics_client_id] = 0;
	nb_used_clients_--;

	return true;
}

b3PhysicsClientHandle PhysicsServers::getPhysicsClient(int physics_client_id)
{
	if ((physics_client_id < 0) || (physics_client_id >= MAX_PHYSICS_CLIENTS) || (0 == clients_handles[physics_client_id]))
		return 0;

	b3PhysicsClientHandle sm = clients_handles[physics_client_id];
	if (sm)
	{
		if (b3CanSubmitCommand(sm))
			return sm;
		else
		{
			//broken connection?
			b3DisconnectSharedMemory(sm);
			clients_handles[physics_client_id] = 0;
			clients_method[physics_client_id] = 0;

			nb_used_clients_--;
		}
	}
	return 0;
}

} // namesapce owds