import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose

# This method calculates the pose of "pose2" in the frame of "pose1"
def calculate_relative_pose(pose1: Pose, pose2: Pose) -> Pose:
    p1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
    p2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])

    # Extract orientations (quaternions)
    q1 = np.array([pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w])
    q2 = np.array([pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w])

    # Convert quaternions to rotation matrices
    R1 = Rotation.from_quat(q1).as_matrix()
    R2 = Rotation.from_quat(q2).as_matrix()

    # Compute the inverse of R1 and apply it to p2-p1 to get relative translation
    p_rel = R1.T @ (p2 - p1)

    # Compute relative rotation
    R_rel = R1.T @ R2
    q_rel = Rotation.from_matrix(R_rel).as_quat()  # Convert back to quaternion

    # Create new Pose message
    rel_pose = Pose()
    rel_pose.position.x, rel_pose.position.y, rel_pose.position.z = p_rel
    rel_pose.orientation.x, rel_pose.orientation.y, rel_pose.orientation.z, rel_pose.orientation.w = q_rel

    return rel_pose

def add_zero_mean_gaussian_noise_to_pose(pose: Pose, stddev_translation: float, stddev_rotation: float) -> Pose:
    """
    Adds Gaussian noise to a Pose message.

    Args:
        pose (Pose): The original pose.
        stddev_translation (float): Standard deviation for translation noise, in meters.
        stddev_rotation (float): Standard deviation for rotation noise, specifically on 
                        the euler angles, in radians.
    
    Returns:
        Pose: The noisy pose.
    """

    # Add translation noise
    noisy_pose = Pose()
    noise_T = np.random.normal(0, stddev_translation, 3)  # 3 for position, 3 for orientation
    noisy_pose.position.x = pose.position.x + noise_T[0]
    noisy_pose.position.y = pose.position.y + noise_T[1]
    noisy_pose.position.z = pose.position.z + noise_T[2]

    # Extract quaternion and convert to euler angles
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    r = Rotation.from_quat(q)
    euler_angles = r.as_euler('xyz', degrees=False)

    # Add rotation noise
    noise_R = np.random.normal(0, stddev_rotation, 3)
    noisy_euler_angles = euler_angles + noise_R

    # Convert back to quaternion
    noisy_rotation = Rotation.from_euler('xyz', noisy_euler_angles).as_quat()
    noisy_pose.orientation.x = noisy_rotation[0]
    noisy_pose.orientation.y = noisy_rotation[1]
    noisy_pose.orientation.z = noisy_rotation[2]
    noisy_pose.orientation.w = noisy_rotation[3]
    
    return noisy_pose