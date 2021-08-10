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
    std::lock_guard<std::mutex> lock(mutex_);
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
    return createVisualShape(GEOM_MESH, 0, {0,0,0}, 0, getFullPath(file_name), scale, rgba_color);
}

int BulletClient::createVisualShape(BulletShapeType_e shape_type, 
                                    float radius,
                                    const std::array<double, 3>& half_extents,
                                    float height,
                                    const std::string& file_name, 
                                    const std::array<double, 3>& mesh_scale,
                                    const std::array<double, 4>& rgba_color)
{
    std::lock_guard<std::mutex> lock(mutex_);
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
    return createCollisionShape(GEOM_MESH, 0, {0}, 0, getFullPath(file_name), scale, flags);
}

int BulletClient::createCollisionShape(BulletShapeType_e shape_type, 
                                        float radius,
                                        const std::array<double, 3>& half_extents,
                                        float height,
                                        const std::string& file_name, 
                                        const std::array<double, 3>& mesh_scale,
                                        int flags)
{
    std::lock_guard<std::mutex> lock(mutex_);
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
    std::lock_guard<std::mutex> lock(mutex_);
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
    std::lock_guard<std::mutex> lock(mutex_);
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

int BulletClient::loadURDFRaw(const std::string& raw_urdf, const std::string& temp_file_name,
                           const std::array<double, 3>& base_position,
                           const std::array<double, 4>& base_orientation,
                           bool use_fixed_base,
                           int flags)
{
    std::ofstream out_file;
    out_file.open(additional_path_ + "/" + temp_file_name + ".owds");
    out_file << raw_urdf;
    out_file.close();
    return loadURDF(temp_file_name + ".owds", base_position, base_orientation, use_fixed_base, flags);

}

// Return the number of joints in an object based on
// body index; body index is based on order of sequence
// the object is loaded into simulation
int BulletClient::getNumJoints(int body_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
	return b3GetNumJoints(*client_handle_, body_id);
}

// Initalize all joint positions given a list of values
bool BulletClient::resetJointState(int body_id, int joint_index, double target_value, double target_velocity)
{
    std::lock_guard<std::mutex> lock(mutex_);
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
    std::lock_guard<std::mutex> lock(mutex_);
	b3SharedMemoryCommandHandle command_handle = b3CreatePoseCommandInit(*client_handle_, body_id);

    b3CreatePoseCommandSetBasePosition(command_handle, position[0], position[1], position[2]);
    b3CreatePoseCommandSetBaseOrientation(command_handle, orientation[0], orientation[1],
                                          orientation[2], orientation[3]);

    b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
}

long BulletClient::createUserConstraint(int parent_body_id, int parent_link_index,
                                        int child_body_id, int child_link_index,
                                        JointType joint_type,
                                        const std::array<double, 3>& joint_axis,
                                        const std::array<double, 3>& parent_frame_pose,
                                        const std::array<double, 3>& child_frame_pose,
                                        const std::array<double, 4>& parent_frame_orientation,
                                        const std::array<double, 4>& child_frame_orientation)
{
    std::lock_guard<std::mutex> lock(mutex_);
    struct b3JointInfo joint_info;
	joint_info.m_jointType = joint_type;
	joint_info.m_parentFrame[0] = parent_frame_pose[0];
	joint_info.m_parentFrame[1] = parent_frame_pose[1];
	joint_info.m_parentFrame[2] = parent_frame_pose[2];
	joint_info.m_parentFrame[3] = parent_frame_orientation[0];
	joint_info.m_parentFrame[4] = parent_frame_orientation[1];
	joint_info.m_parentFrame[5] = parent_frame_orientation[2];
	joint_info.m_parentFrame[6] = parent_frame_orientation[3];

	joint_info.m_childFrame[0] = child_frame_pose[0];
	joint_info.m_childFrame[1] = child_frame_pose[1];
	joint_info.m_childFrame[2] = child_frame_pose[2];
	joint_info.m_childFrame[3] = child_frame_orientation[0];
	joint_info.m_childFrame[4] = child_frame_orientation[1];
	joint_info.m_childFrame[5] = child_frame_orientation[2];
	joint_info.m_childFrame[6] = child_frame_orientation[3];

	joint_info.m_jointAxis[0] = joint_axis[0];
	joint_info.m_jointAxis[1] = joint_axis[1];
	joint_info.m_jointAxis[2] = joint_axis[2];

	b3SharedMemoryCommandHandle command_handle = b3InitCreateUserConstraintCommand(*client_handle_, parent_body_id, parent_link_index, child_body_id, child_link_index, &joint_info);
	b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
	int status_type = b3GetStatusType(status_handle);
	if (status_type == CMD_USER_CONSTRAINT_COMPLETED)
        return b3GetStatusUserConstraintUniqueId(status_handle);

	ShellDisplay::error("createConstraint failed.");
	return -1;
}

void BulletClient::removeUserConstraint(int user_constraint_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
	b3SharedMemoryCommandHandle command_handle = b3InitRemoveUserConstraintCommand(*client_handle_, user_constraint_id);
	b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
}

void BulletClient::changeUserConstraint(int user_constraint_id,
                         const std::array<double, 3>& joint_child_pivot,
                         const std::array<double, 4>& joint_child_frame_orientation,
                         double max_force)
{
    std::lock_guard<std::mutex> lock(mutex_);
	b3SharedMemoryCommandHandle command_handle = b3InitChangeUserConstraintCommand(*client_handle_, user_constraint_id);

    b3InitChangeUserConstraintSetPivotInB(command_handle, &joint_child_pivot[0]);
    b3InitChangeUserConstraintSetFrameInB(command_handle, &joint_child_frame_orientation[0]);

	if (max_force >= 0)
		b3InitChangeUserConstraintSetMaxForce(command_handle, max_force);
	
	b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
}

// use the enum DynamicsActivationState for the activation_state
void BulletClient::changeDynamicsInfo(int body_id, int link_index, int friction_anchor, int activation_state)
{
    std::lock_guard<std::mutex> lock(mutex_);
    b3SharedMemoryCommandHandle command_handle = b3InitChangeDynamicsInfo(*client_handle_);

    if (friction_anchor >= 0)
        b3ChangeDynamicsInfoSetFrictionAnchor(command_handle, body_id, link_index, friction_anchor);

    if (activation_state >= 0)
        b3ChangeDynamicsInfoSetActivationState(command_handle, body_id, activation_state);        
            
    b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
}

std::array<float, 16> BulletClient::computeProjectionMatrix(float fov,
                                                            float aspect,
                                                            float near_value,
                                                            float far_value)
{
    std::lock_guard<std::mutex> lock(mutex_);
	std::array<float, 16> projection_matrix;
	b3ComputeProjectionMatrixFOV(fov, aspect, near_value, far_value, &projection_matrix[0]);
	return projection_matrix;
}

std::array<float, 16> BulletClient::computeProjectionMatrix(float left,
                                                            float right,
                                                            float bottom,
                                                            float top,
                                                            float near_value,
                                                            float far_value)
{
    std::lock_guard<std::mutex> lock(mutex_);
	std::array<float, 16> projection_matrix;
	b3ComputeProjectionMatrix(left, right, bottom, top, near_value, far_value, &projection_matrix[0]);
	return projection_matrix;
}

std::array<float, 16> BulletClient::computeProjectionMatrix(const std::array<float, 3>& camera_position,
                                                            float distance,
                                                            float yaw,
                                                            float pitch,
                                                            float roll,
                                                            int up_axis_index)
{
    std::lock_guard<std::mutex> lock(mutex_);
	std::array<float, 16> projection_matrix;
	b3ComputeViewMatrixFromYawPitchRoll(&camera_position[0], distance, yaw, pitch, roll, up_axis_index, &projection_matrix[0]);
	return projection_matrix;
}

std::array<float, 16> BulletClient::computeViewMatrix(const std::array<float, 3>& camera_eye_position,
                                                      const std::array<float, 3>& camera_target_position,
                                                      const std::array<float, 3>& camera_up_vector)
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::array<float, 16> view_matrix;
	b3ComputeViewMatrixFromPositions(&camera_eye_position[0], &camera_target_position[0], &camera_up_vector[0], &view_matrix[0]);
    return view_matrix;
}

std::array<float, 16> BulletClient::computeViewMatrix(const std::array<float, 3>& camera_target_position,
                                                      float distance,
                                                      float yaw,
                                                      float pitch,
                                                      float roll,
                                                      int up_axis_index)
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::array<float, 16> view_matrix;
	b3ComputeViewMatrixFromYawPitchRoll(&camera_target_position[0], distance, yaw, pitch, roll, up_axis_index, &view_matrix[0]);
    return view_matrix;
}

struct b3CameraImageData BulletClient::getCameraImage(int width, int height,
                                                      std::array<float, 16>& view_matrix, 
                                                      std::array<float, 16>& projection_matrix,
                                                      Renderer_e renderer,
                                                      int flags)
{
    std::lock_guard<std::mutex> lock(mutex_);
	b3SharedMemoryCommandHandle command = b3InitRequestCameraImage(*client_handle_);
	b3RequestCameraImageSetPixelResolution(command, width, height);

	b3RequestCameraImageSetCameraMatrices(command, &view_matrix[0], &projection_matrix[0]);
	if (flags >= 0)
		b3RequestCameraImageSetFlags(command, flags);
    b3RequestCameraImageSetShadow(command, 0);

	b3RequestCameraImageSelectRenderer(command, renderer);

	if (b3CanSubmitCommand(*client_handle_))
	{
		b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command);
		int status_type = b3GetStatusType(status_handle);
		if (status_type == CMD_CAMERA_IMAGE_COMPLETED)
		{

            struct b3CameraImageData image_data;
			b3GetCameraImageData(*client_handle_, &image_data);
            return image_data;
		}
        else
            ShellDisplay::error("getCameraImage failed");
	}
    else
        ShellDisplay::error("getCameraImage can not submit the command");

	struct b3CameraImageData image_data_empty;
    image_data_empty.m_pixelHeight = 0;
    image_data_empty.m_pixelWidth = 0;
    image_data_empty.m_depthValues = nullptr;
    image_data_empty.m_rgbColorData = nullptr;
    image_data_empty.m_segmentationMaskValues = nullptr;
	return image_data_empty;
}

std::unordered_set<int> BulletClient::getSegmentationIds(const b3CameraImageData& image)
{
  std::unordered_set<int> segmentation_ids;
  for(size_t i = 0; i < image.m_pixelHeight*image.m_pixelWidth; i++)
    segmentation_ids.insert(image.m_segmentationMaskValues[i]);

  return segmentation_ids;
}

void BulletClient::configureDebugVisualizer(b3ConfigureDebugVisualizerEnum flag, bool enable)
{
    std::lock_guard<std::mutex> lock(mutex_);
    b3SharedMemoryCommandHandle command_handle = b3InitConfigureOpenGLVisualizer(*client_handle_);
    b3ConfigureOpenGLVisualizerSetVisualizationFlags(command_handle, flag, enable);
    b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
}

struct b3VisualShapeInformation BulletClient::getVisualShapeData(int object_id, int flags)
{
    std::lock_guard<std::mutex> lock(mutex_);
    b3SharedMemoryCommandHandle command_handle = b3InitRequestVisualShapeInformation(*client_handle_, object_id);
    b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
    int status_type = b3GetStatusType(status_handle);
    if (status_type == CMD_VISUAL_SHAPE_INFO_COMPLETED)
    {
        struct b3VisualShapeInformation visual_shape_info;
        b3GetVisualShapeInformation(*client_handle_, &visual_shape_info);
        return visual_shape_info;
    }
    else
    {
        ShellDisplay::error("Error receiving visual shape info");
        struct b3VisualShapeInformation visual_shape_info_empty;
        visual_shape_info_empty.m_numVisualShapes = 0;
        visual_shape_info_empty.m_visualShapeData = nullptr;
        return visual_shape_info_empty;
    }
}

struct b3LinkState BulletClient::getLinkState(int body_id, int link_index, bool compute_link_velocity, bool compute_forward_kinematics)
{
    std::lock_guard<std::mutex> lock(mutex_);
    struct b3LinkState link_state_empty;

    if (body_id < 0)
    {
        ShellDisplay::error("getLinkState failed; invalid bodyUniqueId");
        return link_state_empty;
    }
    else if (link_index < 0)
    {
        ShellDisplay::error("getLinkState failed; invalid linkIndex");
        return link_state_empty;
    }

    b3SharedMemoryCommandHandle command_handle = b3RequestActualStateCommandInit(*client_handle_, body_id);

    if (compute_link_velocity)
        b3RequestActualStateCommandComputeLinkVelocity(command_handle, compute_link_velocity);

    if (compute_forward_kinematics)
        b3RequestActualStateCommandComputeForwardKinematics(command_handle, compute_forward_kinematics);

    b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
    {
        ShellDisplay::error("getLinkState failed.");
        return link_state_empty;
    }

    struct b3LinkState link_state;
    if (b3GetLinkState(*client_handle_, status_handle, link_index, &link_state))
        return link_state;

	return link_state_empty;
}

struct b3JointInfo BulletClient::getJointInfo(int body_id, int joint_index)
{
    std::lock_guard<std::mutex> lock(mutex_);
	struct b3JointInfo info;

    if (b3GetJointInfo(*client_handle_, body_id, joint_index, &info))
        return info;
    else
    {
        ShellDisplay::error("GetJointInfo failed.");
        return info;
    }
}

std::pair<std::unordered_map<std::string, int>, std::unordered_map<std::string, int>> BulletClient::findJointAndLinkIndices(int body_id)
{
    std::unordered_map<std::string, int> joint_name_index;
    std::unordered_map<std::string, int> link_name_index;
    int numJoints = getNumJoints(body_id);
    std::cout << "Robot id: " << body_id << " Num_Joints : " << numJoints << std::endl;
    if (numJoints == 0)
    {
        std::cout << "Warning: No joints found for Bullet body id: " << body_id << std::endl;
        return {joint_name_index, link_name_index};
    }
    for (size_t i = 0; i < numJoints - 1; i++)
    {
        b3JointInfo joint = getJointInfo(body_id, i);
        std::cout << joint.m_jointName << std::endl;
        joint_name_index[joint.m_jointName] = i;
        link_name_index[joint.m_linkName] = i;
    }
    return {joint_name_index, link_name_index};
}

long BulletClient::addUserDebugLine(const std::array<double, 3>& xyz_from,
                                    const std::array<double, 3>& xyz_to,
                                    const std::array<double, 3>& color_rgb,
                                    double line_width,
                                    double life_time,
                                    int replace_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
	b3SharedMemoryCommandHandle command_handle = b3InitUserDebugDrawAddLine3D(*client_handle_, 
                                                                              &xyz_from[0],
                                                                              &xyz_to[0],
                                                                              &color_rgb[0],
                                                                              line_width, life_time);

	if (replace_id >= 0)
		b3UserDebugItemSetReplaceItemUniqueId(command_handle, replace_id);

	b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
	int status_type = b3GetStatusType(status_handle);
	if (status_type == CMD_USER_DEBUG_DRAW_COMPLETED)
        return b3GetDebugItemUniqueId(status_handle);
    else
    {
        ShellDisplay::error("failed to draw debug line");
        return -1;
    }
}

std::vector<struct b3RayHitInfo> BulletClient::rayTestBatch(const std::vector<std::array<double, 3>>& from_poses,
                                                            const std::vector<std::array<double,3>>& to_poses,
                                                            int nb_thread,
                                                            bool report_hit_number)
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<struct b3RayHitInfo> raycast_info_res;

	b3SharedMemoryCommandHandle command_handle = b3CreateRaycastBatchCommandInit(*client_handle_);
	b3RaycastBatchSetNumThreads(command_handle, nb_thread);

    if (from_poses.size() != to_poses.size())
    {
        ShellDisplay::error("Size of from_positions need to be equal to size of to_positions.");
        return raycast_info_res;
    }
    else
    {
        if (from_poses.size() > MAX_RAY_INTERSECTION_BATCH_SIZE_STREAMING)
        {
            ShellDisplay::error("Number of rays exceed the maximum batch size.");
            return raycast_info_res;
        }

        for (int i = 0; i < from_poses.size(); i++)
            b3RaycastBatchAddRays(*client_handle_, command_handle, &from_poses[i][0], &to_poses[i][0], 1);
    }

	if (report_hit_number)
		b3RaycastBatchSetReportHitNumber(command_handle, report_hit_number);

	b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
	int status_type = b3GetStatusType(status_handle);
	if (status_type == CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED)
	{
		struct b3RaycastInformation raycast_info;
		b3GetRaycastInformation(*client_handle_, &raycast_info);

        for(size_t i = 0; i < raycast_info.m_numRayHits; i++)
        {
            if((raycast_info.m_rayHits[i].m_hitObjectUniqueId != -1) ||
               (raycast_info.m_rayHits[i].m_hitObjectLinkIndex != -1))
            {
                raycast_info_res.push_back(raycast_info.m_rayHits[i]);
            }
        }

		return raycast_info_res;
	}

	return raycast_info_res;
}

// perform collision detection: update aabbs, compute overlapping pairs and contact points
void BulletClient::performCollisionDetection()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (b3CanSubmitCommand(*client_handle_))
        b3SubmitClientCommandAndWaitStatus(*client_handle_, b3InitPerformCollisionDetectionCommand(*client_handle_));
    else
        ShellDisplay::warning("Collisions not updated");
}

struct aabb_t BulletClient::getAABB(int body_id, int link_index)
{
    std::lock_guard<std::mutex> lock(mutex_);
    struct aabb_t aabb;
    aabb.is_valid = false;

    if (body_id < 0)
    {
        ShellDisplay::error("getAABB failed; invalid body_id");
        return aabb;
    }

    if (link_index < -1)
    {
        ShellDisplay::error("getAABB failed; invalid link_index");
        return aabb;
    }

    b3SharedMemoryCommandHandle command_handle = b3RequestCollisionInfoCommandInit(*client_handle_, body_id);
    b3SharedMemoryStatusHandle status_handle = b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_REQUEST_COLLISION_INFO_COMPLETED)
    {
        ShellDisplay::error("getAABB failed.");
        return aabb;
    }

    if (b3GetStatusAABB(status_handle, link_index, &aabb.min[0], &aabb.max[0]))
    {
        aabb.is_valid = true;
        return aabb;
    }

    return aabb;  
}

struct b3AABBOverlapData BulletClient::getOverlappingObjects(const struct aabb_t& aabb)
{
    std::lock_guard<std::mutex> lock(mutex_);
	struct b3AABBOverlapData overlap_data;

	b3SharedMemoryCommandHandle command_handle = b3InitAABBOverlapQuery(*client_handle_, &aabb.min[0], &aabb.max[0]);
	b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
	b3GetAABBOverlapResults(*client_handle_, &overlap_data);
    return overlap_data;
}

void BulletClient::resetDebugVisualizerCamera(float distance, float yaw, float pitch, const std::array<float,3>& target_pose)
{
	{
		b3SharedMemoryCommandHandle commandHandle = b3InitConfigureOpenGLVisualizer(*client_handle_);
		if ((distance >= 0))
		{
			b3ConfigureOpenGLVisualizerSetViewMatrix(commandHandle, distance, pitch, yaw, &target_pose[0]);
		}
		b3SubmitClientCommandAndWaitStatus(*client_handle_, commandHandle);
	}
}

b3MouseEventsData BulletClient::getMouseEvents()
{
    std::lock_guard<std::mutex> lock(mutex_);
	struct b3MouseEventsData mouseEventsData;
	b3SharedMemoryCommandHandle command_handle = b3RequestMouseEventsCommandInit(*client_handle_);
	b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
	b3GetMouseEventsData(*client_handle_, &mouseEventsData);
	return mouseEventsData;
}

b3KeyboardEventsData BulletClient::getKeyboardEvents()
{
    std::lock_guard<std::mutex> lock(mutex_);
	struct b3KeyboardEventsData keyboardEvent;
	b3SharedMemoryCommandHandle command_handle = b3RequestKeyboardEventsCommandInit(*client_handle_);
	b3SubmitClientCommandAndWaitStatus(*client_handle_, command_handle);
	b3GetKeyboardEventsData(*client_handle_, &keyboardEvent);
	return keyboardEvent;
}

} // namespace owds