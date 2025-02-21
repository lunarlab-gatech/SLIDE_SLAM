#!/usr/bin/env python3
# This file is a modified version of the original code from: https://github.com/PRBonn/lidar-bonnetal

from load_model import Load_Model
from laserscan import LaserScan
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import torch
import torch.backends.cudnn as cudnn


import ros_numpy


class Inference:
    def __init__(self, node_name):
        self.node_name = node_name
        ################################ IMPORTANT PARAMS ################################
        self.desired_frequency = rospy.get_param(
            "/"+self.node_name+"/desired_frequency", default=2)  # 0 means no limit
        num_cpu_threads = rospy.get_param(
            "/"+self.node_name+"/num_cpu_threads", default=10)
        self.range_threshold = rospy.get_param(
            "/"+self.node_name+"/pc_range_threshold", default=20)  # meters
        self.out_of_range_pts_default_position = np.array(rospy.get_param(
            "/"+self.node_name+"/out_of_range_default_position", default=[0, 0, 0]))
        self.pc_point_step = rospy.get_param("/"+self.node_name+"/pc_point_step", default=16)

        torch.set_num_threads(num_cpu_threads)

        model_directory = rospy.get_param(
            "/"+self.node_name+"/model_dir", default="/home/sam/semantic-segmentation/lidar-bonnetal/pennovation-darknet-smallest/")
        namespace = rospy.get_param(
            "/"+self.node_name+"/namespace", default="/os_node")
        print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print(f"[infer_node] desired frequency for inference is: ", self.desired_frequency,
              " Hz. If its a positive value, some point clouds will be skipped for inference. Set to 0 to run inference on every point cloud.")
        print(f"[infer_node] using namespace: ", namespace,
              " to publish segmented point cloud (should be /quadrotor if running on the robot, otherwise /os_node)")
        print(f"[infer_node] setting number of CPU threads to: ", num_cpu_threads,
              " for Pytorch CPU inference (should be small if you are running on the robot)")
        print(f"[infer_node] setting range threshold for final segmented point cloud (for cutting off all far-away points) to: ", self.range_threshold,
              " meters. Those cut-off points will be set to have coordinate: ", self.out_of_range_pts_default_position)
        print(f"[infer_node] model directory is: ", model_directory)
        print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        ################################ IMPORTANT PARAMS ENDS ################################

        self.last_called_stamp = None

        self.load_obj_ = Load_Model(model_directory)
        self.model_ = self.load_obj_.load_model()
        self.scan_obj_ = LaserScan(project=True,
                                   H=self.load_obj_.arch_configs["dataset"]["sensor"]["img_prop"]["height"],
                                   W=self.load_obj_.arch_configs["dataset"]["sensor"]["img_prop"]["width"],
                                   fov_up=self.load_obj_.arch_configs["dataset"]["sensor"]["fov_up"],
                                   fov_down=self.load_obj_.arch_configs["dataset"]["sensor"]["fov_down"],
                                   range_threshold=self.range_threshold)

        # Set model cuda parameters
        self.gpu_ = False
        if torch.cuda.is_available() and torch.cuda.device_count() > 0 and \
                rospy.get_param("gpu", default=True):
            cudnn.benchmark = True
            cudnn.fastest = True
            self.gpu_ = True
            self.model_.cuda()
        else:
            self.model_.cpu()

        # Push model into eval mode
        self.model_.eval()
        if self.gpu_:
            torch.cuda.empty_cache()

        self.info_ = None
        self.pc_fields_ = self.make_fields()

        # Subscriber for using point cloud from Faster-LIO
        self.scan_sub_ = rospy.Subscriber(
            "/cloud_registered_body", PointCloud2, callback=self.pc_cb, queue_size=1)
        # Publisher for publishing segmented point cloud
        self.pc_pub_ = rospy.Publisher(
            namespace+"/segmented_point_cloud_no_destagger", PointCloud2, queue_size=1)
        # Publisher for active SLAM to publish car probability. Currently not used
        # self.prob_pub = rospy.Publisher(namespace + "/os_node/segmented_point_cloud_no_destagger/car_prob", PointCloud2, queue_size=1)

    def make_fields(self):
        fields = []
        field = PointField()
        field.name = 'x'
        field.count = 1
        field.offset = 0
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'y'
        field.count = 1
        field.offset = 4
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'z'
        field.count = 1
        field.offset = 8
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'intensity'
        field.count = 1
        field.offset = 12
        field.datatype = PointField.FLOAT32
        fields.append(field)
        return fields

    def pc_cb(self, msg):
        if self.last_called_stamp is not None:
            if self.desired_frequency > 0:
                if (msg.header.stamp.to_sec() - self.last_called_stamp.to_sec()) > (1.0 / self.desired_frequency):
                    self.last_called_stamp = msg.header.stamp
                else:
                    rospy.logwarn_throttle(
                        10, "Skipping this point cloud for inference as desired frequency is {} Hz. Set desired frequency to 0 to run inference on every point cloud.".format(self.desired_frequency))
                    return
            else:
                rospy.logwarn_throttle(
                    10, "Desired frequency is set to 0. Running inference on every point cloud. Set desired frequency to a positive value to limit the inference rate.")
        else:
            self.last_called_stamp = msg.header.stamp

        # store the header in case it get updated before the inference finishes
        header = msg.header

        # If destaggering is needed
        # ----------------------------------------------------------
        # scan_data = np.frombuffer(msg.data, dtype=np.float32).reshape(
        #     self.info_.height, self.info_.width, 4).copy()
        # destagger
        # for row, shift in enumerate(self.info_.D):
        #     scan_data[row, :, :] = np.roll(
        #         scan_data[row, :, :], int(shift), axis=0)
        # points_xyz = np.nan_to_num(scan_data[:, :, :3], nan=0.0).reshape(-1, 3)
        # points_intensity = np.frombuffer(scan_data[:, :, 3].tobytes(), dtype=np.uint16).reshape(
        #     points_xyz.shape[0], -1)[:, 1].astype(np.float32)
        # ----------------------------------------------------------

        #undistorted_pc = None
        # try:
        undistorted_pc = ros_numpy.numpify(msg)
        # except ValueError as e:
        #     rospy.loginfo_throttle(10, f"Height: {msg.height}")
        #     rospy.loginfo_throttle(10, f"width: {msg.width}")
        #     rospy.loginfo_throttle(10, f"Point Step: {msg.point_step}")
        #     rospy.loginfo_throttle(10, f"Row Step: {msg.row_step}")
        #     rospy.loginfo_throttle(10, f"Buffer Size: {len(msg.data)}")
        #     rospy.logerr_throttle(10, f"Error with numpify: {e}")
        #     return 

        self.pc_width = undistorted_pc['x'].flatten().shape[0]
        self.pc_height = 1
        points_xyz = np.zeros((undistorted_pc['x'].flatten().shape[0], 3))
        points_xyz[:, 0] = undistorted_pc['x'].flatten()
        points_xyz[:, 1] = undistorted_pc['y'].flatten()
        points_xyz[:, 2] = undistorted_pc['z'].flatten()
        points_intensity = (undistorted_pc['intensity']).flatten()

        # Load into the LaserScan class to do inference
        self.scan_obj_.open_scan(points_xyz=points_xyz.copy(),
                                 points_intensity=points_intensity)

        # make a tensor of the uncompressed data (with the max num points)
        unproj_n_points = self.scan_obj_.points.shape[0]
        unproj_xyz = torch.full(
            (self.load_obj_.arch_configs["dataset"]["max_points"], 3), -1.0, dtype=torch.float)

        unproj_xyz[:unproj_n_points] = torch.from_numpy(self.scan_obj_.points)
        unproj_range = torch.full(
            [self.load_obj_.arch_configs["dataset"]["max_points"]], -1.0, dtype=torch.float)
        unproj_range[:unproj_n_points] = torch.from_numpy(
            self.scan_obj_.unproj_range)
        unproj_remissions = torch.full(
            [self.load_obj_.arch_configs["dataset"]["max_points"]], -1.0, dtype=torch.float)
        unproj_remissions[:unproj_n_points] = torch.from_numpy(
            self.scan_obj_.remissions)

        # prepare final points
        proj_range = torch.from_numpy(self.scan_obj_.proj_range).clone()
        # projected point cloud xyz - [H,W,3] xyz coord (-1 is no data)
        proj_xyz = torch.from_numpy(self.scan_obj_.proj_xyz).clone()
        proj_remission = torch.from_numpy(
            self.scan_obj_.proj_remission).clone()
        proj_mask = torch.from_numpy(self.scan_obj_.proj_mask)

        proj_x = torch.full([self.load_obj_.arch_configs["dataset"]
                            ["max_points"]], -1, dtype=torch.long)
        proj_x[:unproj_n_points] = torch.from_numpy(self.scan_obj_.proj_x)
        proj_y = torch.full([self.load_obj_.arch_configs["dataset"]
                            ["max_points"]], -1, dtype=torch.long)
        proj_y[:unproj_n_points] = torch.from_numpy(self.scan_obj_.proj_y)

        proj = torch.cat([proj_range.unsqueeze(0).clone(),
                          proj_xyz.clone().permute(2, 0, 1),
                          proj_remission.unsqueeze(0).clone()])

        proj = (proj - torch.tensor(self.load_obj_.arch_configs["dataset"]["sensor"]["img_means"])[:, None, None]
                ) / torch.tensor(self.load_obj_.arch_configs["dataset"]["sensor"]["img_stds"])[:, None, None]

        proj = proj * proj_mask.float()

        with torch.no_grad():
            p_x = proj_x[:unproj_n_points]
            p_y = proj_y[:unproj_n_points]
            proj_range = proj_range[0, :unproj_n_points]
            unproj_range = unproj_range[:unproj_n_points]

            proj = torch.unsqueeze(proj, 0)
            proj_mask = torch.unsqueeze(proj_mask, 0)

            if self.gpu_:
                proj_in = proj.cuda()
                proj_mask = proj_mask.cuda()
                p_x = p_x.cuda()
                p_y = p_y.cuda()
            else:
                proj_in = proj
                proj_mask = proj_mask
                p_x = p_x
                p_y = p_y

            proj_output = self.model_(proj_in, proj_mask)
            if self.gpu_:
                torch.cuda.synchronize()

            # Output probability instead of hard classification
            proj_argmax = proj_output[0].argmax(dim=0)

            # threshold out of range points
            points_xyz[np.linalg.norm(
                points_xyz, axis=1) > self.range_threshold, :] = self.out_of_range_pts_default_position

            # Only used for Active SLAM for calculating car class probability
            # --------------------------------------------------------------------------------------------
            # proj_car_prob = proj_output[0][5,:,:]
            # unproj_argmax_car_prob = proj_car_prob[p_y, p_x]
            # pred_np_car_prob = unproj_argmax_car_prob.cpu().numpy()
            # pred_np_car_prob = pred_np_car_prob.reshape((-1))
            # # threshold out of range points
            # pred_np_car_prob[np.linalg.norm(points_xyz, axis = 1) > self.range_threshold] = 0

            # pc_msg = PointCloud2()
            # pc_msg.header = header
            # pc_msg.width = self.pc_width
            # pc_msg.height = self.pc_height
            # pc_msg.point_step = 16
            # pc_msg.row_step = pc_msg.width * pc_msg.point_step
            # pc_msg.fields = self.pc_fields_

            # full_data = np.hstack(
            # (points_xyz, pred_np_car_prob[:, None])).astype(np.float32)
            # pc_msg.data = full_data.tobytes()
            # self.prob_pub.publish(pc_msg)
            # --------------------------------------------------------------------------------------------

            pred_np_range_image = proj_argmax.cpu().numpy()
            pred_np_range_image = pred_np_range_image.reshape((-1))
            proj_xyz_range_image = proj_xyz.cpu().numpy()
            proj_xyz_range_image = proj_xyz_range_image.reshape((-1, 3))
            # threshold out of points
            pred_np_range_image[np.linalg.norm(
                proj_xyz_range_image, axis=1) > self.range_threshold] = 0
            # make sure that points do not exist in the originial input PC are labeled as 0
            pred_np_range_image[proj_xyz_range_image[:, 0] == -1] = 0
            # print(f"number of points that do not have data: ", np.sum(proj_xyz_range_image[:,0] == -1))
            # print(f"number of points that DO have data: ", np.sum(proj_xyz_range_image[:,0] != -1))
            proj_xyz_range_image[proj_xyz_range_image[:, 0] == -1] = np.NaN
            full_data_range_image = np.hstack(
                (proj_xyz_range_image, pred_np_range_image[:, None])).astype(np.float32)

            pc_msg = PointCloud2()
            pc_msg.header = header
            pc_msg.header.frame_id = "body"
            rospy.logwarn_throttle(
                30, "Segmented point cloud is currently hardcoded to be published in \"body\" frame. Please change it to the correct frame if needed.")
            pc_msg.width = self.pc_width
            pc_msg.height = self.pc_height
            pc_msg.point_step = self.pc_point_step
            pc_msg.row_step = pc_msg.width * pc_msg.point_step
            pc_msg.fields = self.pc_fields_
            pc_msg.data = full_data_range_image.tobytes()
            self.pc_pub_.publish(pc_msg)

            # Original code for publishing segmented point cloud where point cloud is already organized
            # --------------------------------------------------------------------------------------------
            # unproj_argmax = proj_argmax[p_y, p_x]
            # pred_np = unproj_argmax.cpu().numpy()
            # pred_np = pred_np.reshape((-1))
            # # threshold out of range points
            # pred_np[np.linalg.norm(points_xyz, axis = 1) > self.range_threshold] = 0
            # pc_msg = PointCloud2()
            # pc_msg.header = header
            # pc_msg.header.frame_id = "body"
            # print('hard coding seg pc frame_id to body')
            # pc_msg.width = self.pc_width
            # pc_msg.height = self.pc_height
            # pc_msg.point_step = 16
            # pc_msg.row_step = pc_msg.width * pc_msg.point_step
            # pc_msg.fields = self.pc_fields_

            # full_data = np.hstack(
            #     (points_xyz, pred_np[:, None])).astype(np.float32)
            # pc_msg.data = full_data.tobytes()
            # self.pc_pub_.publish(pc_msg)
            # --------------------------------------------------------------------------------------------

            rospy.loginfo_throttle(
                5, "Inference done. Segmented point cloud published.")


if __name__ == '__main__':

    node_name = rospy.get_param("/infer_node_name", default="inference_node")
    rospy.init_node(node_name)
    inf = Inference(node_name)

    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo("Shutting down {}...".format(node_name))
