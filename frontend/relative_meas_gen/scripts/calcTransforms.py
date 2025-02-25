import yaml
from scipy.spatial.transform import Rotation as R
import numpy as np

# This function calculates the IMU-to-LIDAR transformation
# for the given extrinsics file (from CoPeD)
def main():
    with open('/opt/slideslam_docker_ws/src/SLIDE_SLAM/bags/CoPeD/CoPeD/calibration/wilbur/extrinsics.yaml', 'r') as file:
        data = yaml.safe_load(file)

        # Construct transformation from sensor_plate_link to imu_link
        sp_to_i = data['imu_joint']
        R_sp_to_i = R.from_euler('XYZ', [sp_to_i['roll'], sp_to_i['pitch'], sp_to_i['yaw']], degrees=False)
        H_sp_to_i = np.eye(4)
        H_sp_to_i[:3, :3] = R_sp_to_i.as_matrix()
        H_sp_to_i[:3, 3] = np.array([sp_to_i['x'], sp_to_i['y'], sp_to_i['z']])

        # Construct transformation from sensor_plate_link to ouster_center_link
        sp_to_oc = data['ouster_center_joint']
        R_sp_to_oc = R.from_euler('XYZ', [sp_to_oc['roll'], sp_to_oc['pitch'], sp_to_oc['yaw']], degrees=False)
        H_sp_to_oc = np.eye(4)
        H_sp_to_oc[:3, :3] = R_sp_to_oc.as_matrix()
        H_sp_to_oc[:3, 3] = np.array([sp_to_oc['x'], sp_to_oc['y'], sp_to_oc['z']])

        # Construct transformation from imu_link to ouster_center_link
        H_i_to_sp = np.linalg.inv(H_sp_to_i)
        H_i_to_oc = H_sp_to_oc @ H_i_to_sp 
        print("Transformation from imu_link to ouster_center_link: ", H_i_to_oc)


if __name__ == "__main__":
    main()