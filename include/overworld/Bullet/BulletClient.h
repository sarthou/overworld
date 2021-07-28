#ifndef OWDS_BULLETCLIENT_H
#define OWDS_BULLETCLIENT_H

#include "SharedMemory/PhysicsClientC_API.h"

#include <string>
#include <array>

namespace owds {

enum ShapeType_e {
    GEOM_SPHERE = 2,
	GEOM_BOX,
	GEOM_CYLINDER,
	GEOM_MESH,
	GEOM_PLANE,
	GEOM_CAPSULE,  //non-standard URDF?
	GEOM_SDF,      //signed-distance-field, non-standard URDF
	GEOM_HEIGHTFIELD,
	GEOM_UNKNOWN
};

class BulletClient
{
public:
    BulletClient(b3PhysicsClientHandle* client_handle, size_t client_id);
    ~BulletClient();

    size_t getId() { return client_id_; }

    void setAdditionalSearchPath(const std::string& path);

    int createVisualShapeBox(const std::array<double, 3>& half_extents, const std::array<double, 4>& rgba_color = {1});
    int createVisualShapeSphere(float radius, const std::array<double, 4>& rgba_color = {1});
    int createVisualShapeCylinder(float radius, float height, const std::array<double, 4>& rgba_color = {1});
    int createVisualShapeCapsule(float radius, float height, const std::array<double, 4>& rgba_color = {1});
    int createVisualShapeMesh(const std::string& file_name, const std::array<double, 3>& scale, const std::array<double, 4>& rgba_color = {1});

    int createCollisionShapeBox(const std::array<double, 3>& half_extents, int flags = 0);
    int createCollisionShapeSphere(float radius, int flags = 0);
    int createCollisionShapeCylinder(float radius, float height, int flags = 0);
    int createCollisionShapeCapsule(float radius, float height, int flags = 0);
    int createCollisionShapeMesh(const std::string& file_name, const std::array<double, 3>& scale, int flags = 0);

    int createMultiBody(float base_mass,
                        int base_collision_shape_index,
                        int base_visual_shape_index,
                        const std::array<double, 3>& base_position,
                        const std::array<double, 4>& base_orientation,
                        int flags = 0);

    int loadURDF(const std::string& file_name,
                const std::array<double, 3>& base_position,
                const std::array<double, 4>& base_orientation,
                bool use_fixed_base = false,
                int flags = 0);
    
private:
    b3PhysicsClientHandle* client_handle_;
    size_t client_id_;

    std::string additional_path_;

    int createVisualShape(ShapeType_e shape_type, 
                            float radius,
                            const std::array<double, 3>& half_extents,
                            float height,
                            const std::string& file_name, 
                            const std::array<double, 3>& mesh_scale,
                            const std::array<double, 4>& rgba_color);

    int createCollisionShape(ShapeType_e shape_type, 
                            float radius,
                            const std::array<double, 3>& half_extents,
                            float height,
                            const std::string& file_name, 
                            const std::array<double, 3>& mesh_scale,
                            int flags);
};

} // namespace owds

#endif // OWDS_BULLETCLIENT_H