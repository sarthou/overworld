#include "overworld/Bullet/BulletClient.h"

#include "overworld/Bullet/PhysicsServers.h"
#include "overworld/Bullet/UrdfHelper.h"
#include "overworld/Utility/ShellDisplay.h"

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

void BulletClient::setAdditionalSearchPath(const std::string& path)
{
	if (path != "")
	{
		b3SharedMemoryCommandHandle command_handle = b3SetAdditionalSearchPath(*client_handle_, path.c_str());
		b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);

        if(additional_path_ != "")
            ShellDisplay::warning("The previous additional path has been overwritten");
        additional_path_ = path;
	}
}

int BulletClient::createVisualShapeBox(const std::array<double, 3>& half_extents, const std::array<double, 4>& rgba_color)
{
    return createVisualShape(GEOM_BOX, 0, half_extents, 0, "", {0}, rgba_color);
}

int BulletClient::createVisualShapeSphere(float radius, const std::array<double, 4>& rgba_color)
{
    return createVisualShape(GEOM_SPHERE, radius, {0}, 0, "", {0}, rgba_color);
}

int BulletClient::createVisualShapeCylinder(float radius, float height, const std::array<double, 4>& rgba_color)
{
    return createVisualShape(GEOM_CYLINDER, radius, {0}, 0, "", {0}, rgba_color);
}

int BulletClient::createVisualShapeCapsule(float radius, float height, const std::array<double, 4>& rgba_color)
{
    return createVisualShape(GEOM_CAPSULE, radius, {0}, height, "", {0}, rgba_color);
}

int BulletClient::createVisualShapeMesh(const std::string& file_name, const std::array<double, 3>& scale, const std::array<double, 4>& rgba_color)
{
    return createVisualShape(GEOM_MESH, 0, {0}, 0, file_name, scale, rgba_color);
}

int BulletClient::createVisualShape(ShapeType_e shape_type, 
                                    float radius,
                                    const std::array<double, 3>& half_extents,
                                    float height,
                                    const std::string& file_name, 
                                    const std::array<double, 3>& mesh_scale,
                                    const std::array<double, 4>& rgba_color)
{
	if (shape_type >= GEOM_SPHERE)
	{
		b3SharedMemoryCommandHandle command_handle = b3CreateVisualShapeCommandInit(*client_handle_);
		int shape_index = -1;

		if (shape_type == GEOM_SPHERE)
		{
            if(radius > 0)
			    shape_index = b3CreateVisualShapeAddSphere(command_handle, radius);
            else
                ShellDisplay::error("Invalid parameters to create sphere visual shape");
		}
		else if (shape_type == GEOM_BOX)
		{
			shape_index = b3CreateVisualShapeAddBox(command_handle, &half_extents[0]);
		}
		else if (shape_type == GEOM_CAPSULE)
		{
            if(radius > 0 && height >= 0)
			    shape_index = b3CreateVisualShapeAddCapsule(command_handle, radius, height);
            else
                ShellDisplay::error("Invalid parameters to create capsule visual shape");
		}
		else if (shape_type == GEOM_CYLINDER)
		{
            if(radius > 0 && height >= 0)
			    shape_index = b3CreateVisualShapeAddCylinder(command_handle, radius, height);
            else
                ShellDisplay::error("Invalid parameters to create cylinder visual shape");
		}
		else if (shape_type == GEOM_MESH)
		{
            if(file_name != "")
			    shape_index = b3CreateVisualShapeAddMesh(command_handle, file_name.c_str(), &mesh_scale[0]);
			else
                ShellDisplay::error("Invalid parameters to create mesh visual shape");
		}

		if (shape_index >= 0)
		{
			b3CreateVisualShapeSetRGBAColor(command_handle, shape_index, &rgba_color[0]);

            double visual_frame_position[3] = {0, 0, 0};
            double visual_frame_orientation[4] = {0, 0, 0, 1};
			b3CreateVisualShapeSetChildTransform(command_handle, shape_index, visual_frame_position, visual_frame_orientation);
		}

		b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
		int status_type = b3GetStatusType(status_handle);
		if (status_type == CMD_CREATE_VISUAL_SHAPE_COMPLETED)
			return b3GetStatusVisualShapeUniqueId(status_handle);
	}
	
    ShellDisplay::error("createVisualShape failed.");
	return -1;
}

int BulletClient::createCollisionShapeBox(const std::array<double, 3>& half_extents, int flags)
{
    return createCollisionShape(GEOM_BOX, 0, half_extents, 0, "", {0}, flags);
}

int BulletClient::createCollisionShapeSphere(float radius, int flags)
{
    return createCollisionShape(GEOM_SPHERE, radius, {0}, 0, "", {0}, flags);
}

int BulletClient::createCollisionShapeCylinder(float radius, float height, int flags)
{
    return createCollisionShape(GEOM_CYLINDER, radius, {0}, 0, "", {0}, flags);
}

int BulletClient::createCollisionShapeCapsule(float radius, float height, int flags)
{
    return createCollisionShape(GEOM_CAPSULE, radius, {0}, height, "", {0}, flags);
}

int BulletClient::createCollisionShapeMesh(const std::string& file_name, const std::array<double, 3>& scale, int flags)
{
    return createCollisionShape(GEOM_MESH, 0, {0}, 0, file_name, scale, flags);
}

int BulletClient::createCollisionShape(ShapeType_e shape_type, 
                                        float radius,
                                        const std::array<double, 3>& half_extents,
                                        float height,
                                        const std::string& file_name, 
                                        const std::array<double, 3>& mesh_scale,
                                        int flags)
{
	if (shape_type >= GEOM_SPHERE)
	{
		int shape_index = -1;

		b3SharedMemoryCommandHandle command_handle = b3CreateCollisionShapeCommandInit(*client_handle_);
		if (shape_type == GEOM_SPHERE && radius > 0)
		{   
            if(radius > 0)
			    shape_index = b3CreateCollisionShapeAddSphere(command_handle, radius);
            else
                ShellDisplay::error("Invalid parameters to create box collision shape");
		}
		else if (shape_type == GEOM_BOX)
		{
			shape_index = b3CreateCollisionShapeAddBox(command_handle, &half_extents[0]);
		}
		else if (shape_type == GEOM_CAPSULE)
		{
            if(radius > 0 && height >= 0)
			    shape_index = b3CreateCollisionShapeAddCapsule(command_handle, radius, height);
            else
                ShellDisplay::error("Invalid parameters to create capsule collision shape");
		}
		else if (shape_type == GEOM_CYLINDER)
		{
            if(radius > 0 && height >= 0)
			    shape_index = b3CreateCollisionShapeAddCylinder(command_handle, radius, height);
            else
                ShellDisplay::error("Invalid parameters to create cylinder collision shape");
		}
		else if (shape_type == GEOM_MESH)
		{
			if(file_name != "")
			    shape_index = b3CreateCollisionShapeAddMesh(command_handle, file_name.c_str(), &mesh_scale[0]);
            else
                ShellDisplay::error("Invalid parameters to create mesh collision shape");
		}

        b3SharedMemoryStatusHandle status_handle;
		int status_type;

		if (shape_index >= 0 && flags)
		{
			b3CreateCollisionSetFlag(command_handle, shape_index, flags);
		}

		status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
		status_type = b3GetStatusType(status_handle);
		if (status_type == CMD_CREATE_COLLISION_SHAPE_COMPLETED)
			return b3GetStatusCollisionShapeUniqueId(status_handle);
	}
	
    ShellDisplay::error("createCollisionShape failed.");
	return -1;
}

int BulletClient::createMultiBody(float base_mass,
                                  int base_collision_shape_index,
                                  int base_visual_shape_index,
                                  const std::array<double, 3>& base_position,
                                  const std::array<double, 4>& base_orientation,
                                  int flags)
{
    b3SharedMemoryCommandHandle command_handle = b3CreateMultiBodyCommandInit(*client_handle_);
    double base_inertial_frame_position[3] = {0, 0, 0};
    double base_inertial_frame_orientation[4] = {0, 0, 0, 1};
    int baseIndex;

    baseIndex = b3CreateMultiBodyBase(command_handle, base_mass,
                                      base_collision_shape_index, base_visual_shape_index,
                                      &base_position[0], &base_orientation[0],
                                      base_inertial_frame_position, base_inertial_frame_orientation);
    if (flags > 0)
    {
        b3CreateMultiBodySetFlags(command_handle, flags);
    }
    b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
    int status_type = b3GetStatusType(status_handle);
    if (status_type == CMD_CREATE_MULTI_BODY_COMPLETED)
            return b3GetStatusBodyIndex(status_handle);

	ShellDisplay::error("createMultiBody failed.");
	return -1;
}

int BulletClient::loadURDF(const std::string& file_name,
                           const std::array<double, 3>& base_position,
                           const std::array<double, 4>& base_orientation,
                           bool use_fixed_base,
                           int flags)
{
	if (file_name != "")
	{
        std::string full_path = file_name;
        if(additional_path_ != "")
            full_path = createLocalUrdf(file_name, additional_path_);
        
		b3SharedMemoryCommandHandle command = b3LoadUrdfCommandInit(*client_handle_, full_path.c_str());
		b3LoadUrdfCommandSetFlags(command, flags);

		b3LoadUrdfCommandSetStartPosition(command, base_position.at(0), base_position.at(1), base_position.at(2));
		b3LoadUrdfCommandSetStartOrientation(command, base_orientation.at(0), base_orientation.at(1),
											 base_orientation.at(2), base_orientation.at(3));

		if (use_fixed_base)
			b3LoadUrdfCommandSetUseFixedBase(command, 1);

		b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command);
		int status_type = b3GetStatusType(status_handle);
		if (status_type != CMD_URDF_LOADING_COMPLETED)
		{
			ShellDisplay::error("Cannot load URDF file.");
			return -1;
		}
		return b3GetStatusBodyIndex(status_handle);
	}
	else
	{
		ShellDisplay::error("Empty filename, method expects 1, 4 or 8 arguments.");
		return -1;
	}
}

// Return the number of joints in an object based on
// body index; body index is based on order of sequence
// the object is loaded into simulation
int BulletClient::getNumJoints(int body_id)
{
	return b3GetNumJoints(*client_handle_, body_id);
}

// Initalize all joint positions given a list of values
bool BulletClient::resetJointState(int body_id, int joint_index, double target_value, double target_velocity)
{
    int nb_joints = b3GetNumJoints(*client_handle_, body_id);
    if ((joint_index >= nb_joints) || (joint_index < 0))
    {
        ShellDisplay::error("Joint index out-of-range.");
        return false;
    }

    b3SharedMemoryCommandHandle command_handle = b3CreatePoseCommandInit(*client_handle_, body_id);

    b3CreatePoseCommandSetJointPosition(*client_handle_, command_handle, joint_index,
                                        target_value);

    b3CreatePoseCommandSetJointVelocity(*client_handle_, command_handle, joint_index,
                                        target_velocity);

    b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
    return true;
}

// Reset the position and orientation of the base/root link, position [x,y,z]
// and orientation quaternion [x,y,z,w]
void BulletClient::resetBasePositionAndOrientation(int body_id, const std::array<double, 3>& position, const std::array<double, 4>& orientation)
{
	b3SharedMemoryCommandHandle command_handle = b3CreatePoseCommandInit(*client_handle_, body_id);

    b3CreatePoseCommandSetBasePosition(command_handle, position[0], position[1], position[2]);
    b3CreatePoseCommandSetBaseOrientation(command_handle, orientation[0], orientation[1],
                                          orientation[2], orientation[3]);

    b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
}

void BulletClient::removeUserConstraint(int user_constraint_id)
{
	b3SharedMemoryCommandHandle command_handle = b3InitRemoveUserConstraintCommand(*client_handle_, user_constraint_id);
	b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
};

void BulletClient::changeUserConstraint(int user_constraint_id,
                         const std::array<double, 3>& joint_child_pivot,
                         const std::array<double, 4>& joint_child_frame_orientation,
                         double max_force)
{
	b3SharedMemoryCommandHandle command_handle = b3InitChangeUserConstraintCommand(*client_handle_, user_constraint_id);

    b3InitChangeUserConstraintSetPivotInB(command_handle, &joint_child_pivot[0]);
    b3InitChangeUserConstraintSetFrameInB(command_handle, &joint_child_frame_orientation[0]);

	if (max_force >= 0)
		b3InitChangeUserConstraintSetMaxForce(command_handle, max_force);
	
	b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
};

} // namespace owds