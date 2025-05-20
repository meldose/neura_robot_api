#!/usr/bin/env python3
"""
Test the psoe estimation client
"""
import json # imported json
from pathlib import Path # imported json

import yaml # imprted yaml module
import cv2 # imprted cv2 module
import numpy as np # imprted numpy module
import pytransform3d.rotations as ptr # imprted ptr module
import pytransform3d.transformations as ptt # imprted ptt module

import rospkg # imprted rospkg module
import rospy # imprted rospy module
from cv_bridge import CvBridge # imprted CvBridge module
from sensor_msgs.msg import Image, CameraInfo # imprted Image,CameraInfo 
import tf # imprted tf


from cosypose_torch.datasets.object_info_dataset import ObjectInfoDataset
from cosypose_torch.utils.rigid_mesh_database import MeshInfoDataBase

from neurapy_ai.clients import InstanceSegmentationClient
from neura_ai_robot_api.clients.pose_estimation_client import (
    PoseEstimationClient,
)

# created class TestDataPublisher

class TestDataPublisher:
    def __init__(self, image, depth, base_to_cam, K=None):
        bridge = CvBridge()
        self.im_pub = rospy.Publisher(
            "/camera/color/image_raw", Image, queue_size=1
        )

        self.depth_pub = rospy.Publisher(
            "/camera/aligned_depth_to_color/image_raw", Image, queue_size=1
        )
        self.cam_pub = rospy.Publisher(
            "/camera/aligned_depth_to_color/camera_info",
            CameraInfo,
            queue_size=1,
        )
        self.br = tf.TransformBroadcaster()

        self.target_frame = "/base"
        self.base_to_cam = base_to_cam
        cam_r = base_to_cam[:3, :3]
        self.cam_rot_q = ptr.quaternion_from_matrix(cam_r)
        self.cam_tr = base_to_cam[:3, 3]

        self.image_msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        self.image_msg.header.frame_id = "camera_color_optical_frame"
        self.depth_msg = bridge.cv2_to_imgmsg(depth, encoding="passthrough")
        self.cam_msg = self._make_camera_info_msg(image.shape, K)

        rospy.Timer(rospy.Duration(1.0), self._publish)

# created function for publishing

    def _publish(self, event):
        self.br.sendTransform(
            self.cam_tr,
            self.cam_rot_q,
            rospy.Time.now(),
            "base",
            "camera_color_optical_frame",
        )
        self.im_pub.publish(self.image_msg)
        self.depth_pub.publish(self.depth_msg)
        self.cam_pub.publish(self.cam_msg)

# created function for making camera info message

    def _make_camera_info_msg(self, image_shape, K):
        camera_info = CameraInfo()
        # store info without header
        camera_info.header.stamp = rospy.Time.now()
        camera_info.header.seq = 1
        camera_info.width = image_shape[1]
        camera_info.height = image_shape[0]

        if K is None:
            K = [
                919.0804443359375,
                0,
                629.5570068359375,
                0,
                916.7627563476562,
                353.8832092285156,
                0,
                0,
                1,
            ]
        else:
            K = K.reshape(-1)
        camera_info.K = K
        camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        P = [K[0], 0, K[2], 0, 0, K[4], K[5], 0, 0, 0, 1, 0]
        camera_info.P = P
        return camera_info

# created functionf for getting the load_gt

def _load_gt(path, base_to_cam):
    if (path / "test_poses.json").exists():
        gt_data = json.load(open((path / "test_poses.json"), "r"))["poses"]
        poses = []
        for entry in gt_data:
            id = entry["obj_id"]
            o2c_r = np.array(entry["cam_R_m2c"]).reshape(3, 3)
            # positions are in mm but we want meter
            o2c_t = np.array(entry["cam_t_m2c"]) * 0.001
            o2c = ptt.transform_from(o2c_r, o2c_t)

            pose = np.dot(np.linalg.inv(base_to_cam), o2c)
            poses.append({"id": id, "pose": pose, "name": entry["name"]})
        K = np.array(
            [
                919.0804443359375,
                0,
                629.5570068359375,
                0,
                916.7627563476562,
                353.8832092285156,
                0,
                0,
                1,
            ]
        )
    elif (path / "ground_truth.yaml").exists():
        gt_data = yaml.load(
            (path / "ground_truth.yaml").open("r"), Loader=yaml.Loader
        )
        poses = []
        for entry in gt_data:
            id = entry["id"]
            pose = entry["pose"]
            pose[:3, 3] *= 0.001
            pose = np.dot(np.linalg.inv(base_to_cam), pose)
            poses.append({"id": id, "pose": pose, "name": entry["name"]})
            K = entry["K"]
    else:
        print([f for f in path.iterdir()])

    return poses, K

# created function for print result

def print_result(result, return_code, gt_poses, mesh_db):
    print("Return code: ", return_code.value, return_code.message)
    assign, dists_tr, dists_rot, confs = _check_result(
        result, gt_poses, mesh_db
    )
    print(
        "        class        |    assigned class    | dist_tr [mm] | "
        "dist_rot [deg] | confidence"
    )
    print("-" * 91)
    for ind, tr in enumerate(dists_tr):
        class_str = "{:^19}".format(result[ind].object_with_pose.name)
        ind_string = "{:^22}".format(
            gt_poses[assign[ind]]["name"] + "_" + str(assign[ind])
        )
        tr_string = "{:^12}".format(str(np.round(tr, 3)))
        rot_string = "{:^13}".format(str(np.round(dists_rot[ind], 3)))
        conf_string = "{:^10}".format(str(np.round(confs[ind], 1)))
        print(
            f" {class_str} | {ind_string} | {tr_string} | {rot_string} "
            f"| {conf_string} "
        )

# created function for checking the result

def _check_result(detected_objects, gt_poses, mesh_db):
    assign_indices = -1 * np.ones(len(detected_objects), dtype=np.uint8)
    rotation_distances = -1 * np.ones(len(detected_objects))
    translation_distances = -1 * np.ones(len(detected_objects))
    confs = -1 * np.ones(len(detected_objects))
    for det_ind, o in enumerate(detected_objects):
        class_name = o.object_with_pose.name
        min_dist = 10000.0
        confs[det_ind] = o.pose_confidence

        pos = o.object_with_pose.pose.translation
        rot_q = o.object_with_pose.pose.orientation
        rot = ptr.matrix_from_quaternion(rot_q)

        for ind, gt in enumerate(gt_poses):
            if class_name != gt["name"]:
                continue
            # claculate translation distance
            dist_tr = np.linalg.norm(pos - gt["pose"][:3, 3])

            # claculate rotation distance
            # get symmetries
            if class_name in mesh_db.symmetries:
                symmetries = mesh_db.symmetries[class_name]
            else:
                id = gt["id"]
                symmetries = mesh_db.symmetries[f"obj_{id:06d}"]
            gt_pose = np.tile(
                np.expand_dims(gt["pose"], 0), [symmetries.shape[0], 1, 1]
            )
            gt_pose = np.matmul(gt_pose, symmetries)
            R = np.matmul(
                np.tile(np.expand_dims(rot, 0), [symmetries.shape[0], 1, 1]),
                np.transpose(gt_pose[:, :3, :3], axes=[0, 2, 1]),
            )
            tr = np.trace(R, axis1=1, axis2=2)
            # calculate the angle between the two matrices
            theta = (tr - 1) / 2.0
            theta = np.arccos(np.clip(theta, -1, 1))
            theta = np.min(theta)

            if dist_tr < min_dist:
                assign_indices[det_ind] = ind
                rotation_distances[det_ind] = theta * 180 / np.pi
                translation_distances[det_ind] = dist_tr * 1000
                min_dist = dist_tr

    return assign_indices, translation_distances, rotation_distances, confs

# created ufnction for loading the object infos

def _load_object_infos(test_path):
    if (test_path / "models_info.json").exists():
        object_dataset = ObjectInfoDataset(test_path)
        return MeshInfoDataBase.from_object_dataset(object_dataset)
    elif (test_path / "objects_info.yaml").exists():
        with (test_path / "objects_info.yaml").open("r") as info_file:
            info = yaml.load(info_file, Loader=yaml.Loader)
        object_infos = []
        for ob_id, ob in info.items():
            label = ob["name"]
            ob["label"] = label
            ob["mesh_units"] = "mm"

            is_symmetric = False
            # make sure we have symmetry annotatins, even if they are
            # empty
            for k in ("symmetries_discrete", "symmetries_continuous"):
                sym = ob.get(k, [])
                if sym is None:
                    sym = []
                ob[k] = sym
                if len(sym) > 0:
                    is_symmetric = True
            ob["is_symmetric"] = is_symmetric
            object_infos.append(ob)
        return MeshInfoDataBase(object_infos)

# calling the main function

if __name__ == "__main__":
    # read test data
    rospy.init_node("test_pose_estimation")
    rospack = rospkg.RosPack()
    data_path = Path(rospack.get_path("pose_estimation_ros")) / "test"
    test_image_path = data_path / "color_image.jpg"
    image = cv2.imread(test_image_path.as_posix())[:, :, ::-1]
    depth = cv2.imread(str(data_path / "depth_image.png"), cv2.IMREAD_UNCHANGED)
    base_to_cam = np.eye(4)
    ground_truth, K = _load_gt(data_path, base_to_cam)
    K = K.reshape(3, 3)
    mesh_db = _load_object_infos(data_path)

    # start the clients
    client = PoseEstimationClient()
    client.set_model("hrg_industrial")
    segmentation_client = InstanceSegmentationClient()
    segmentation_client.set_model("hrg_industrial")

    # start a node to publish image data on the camera topic
    publisher = TestDataPublisher(image, depth, base_to_cam, K)

    print("test with scene from topic")
    return_code, detected_objects = client.get_poses(
        target_frame="camera_color_optical_frame"
    )
    print_result(detected_objects, return_code, ground_truth, mesh_db)

    # # test get_segmented_instances_from_image
    print("test with input scene")
    return_code, detected_objects = client.get_poses_from_scene(
        image, depth, K, base_to_cam, target_frame="camera_color_optical_frame"
    )
    print_result(detected_objects, return_code, ground_truth, mesh_db)

    # # test get_segmented_instances_from_segmetnation
    print("test with input segmentation")
    segmentation_result = segmentation_client.get_segmented_instances()
    return_code, detected_objects = client.get_poses_from_segmentation_result(
        image,
        depth,
        segmentation_result[1],
        segmentation_result[2],
        K,
        base_to_cam,
        target_frame="camera_color_optical_frame",
    )
    print_result(detected_objects, return_code, ground_truth, mesh_db)

    print("deactivate icp")
    client.set_parameter("icp_refinement", False)
    return_code, detected_objects = client.get_poses_from_scene(
        image, depth, K, base_to_cam, target_frame="camera_color_optical_frame"
    )
    print_result(detected_objects, return_code, ground_truth, mesh_db)

    print("use simple method")
    client.set_parameter("icp_refinement", True)
    client.set_method("simple")
    client.set_model("hrg_industrial")

    return_code, detected_objects = client.get_poses_from_scene(
        image, depth, K, base_to_cam, target_frame="camera_color_optical_frame"
    )
    print_result(detected_objects, return_code, ground_truth, mesh_db)
    client.set_method("cosypose_torch")
    client.set_model("hrg_industrial")
