#!/usr/bin/env python3

import rospy
from typing import List, Tuple
from neura_ai_robot_api.clients.grasp_generator_client import (
    GraspGeneratorClient,
)

from instance_segmentation_ros_msgs.msg import (
    SegmentedInstance as SegmentedInstanceMsg,
)
from pose_estimation_ros_msgs.msg import DetectedObject as DetectedObjectMsg
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import PointCloud2, Image, CameraInfo


class TestGraspGeneratorAPI:
    def __init__(self) -> None:
        # ************* init variables *************
        import message_filters

        self.point_cloud_data = PointCloud2()
        self.rgb_image_data = Image()
        self.depth_image_data = Image()
        self.camera_info_data = CameraInfo()
        point_cloud_sub = message_filters.Subscriber(
            "/camera/depth_registered/points", PointCloud2
        )
        rgb_image_sub = message_filters.Subscriber(
            "/camera/color/image_raw", Image
        )
        depth_image_sub = message_filters.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image
        )
        camera_info_sub = message_filters.Subscriber(
            "/camera/color/camera_info", CameraInfo
        )
        ts = message_filters.ApproximateTimeSynchronizer(
            [point_cloud_sub, rgb_image_sub, depth_image_sub, camera_info_sub],
            10,
            0.1,
        )
        ts.registerCallback(self._get_vision_data_callback)
        self.__vision_data_available = False

    def test_start_detection(
        self, grasp_generator_api: GraspGeneratorClient, object_names=[]
    ):
        start_detection_return_code = grasp_generator_api.start_detection(
            object_names=object_names,
            workspace_name="workspace_2023",
            bin_name="bin_ws_2023",
            gripper_name="RobotiQ_140",
        )

        print(start_detection_return_code)

    def test_generate_grasps_ros(
        self, grasp_generator_api: GraspGeneratorClient, object_names=[]
    ):
        import time

        #####################################################################
        # raw vision data
        cur_time = time.time()
        while (
            time.time() - cur_time < 10
            and self.__vision_data_available == False
        ):
            rospy.loginfo(
                "[databased_grasp_generation]: waiting for vision data to be available"
            )
            time.sleep(0.1)
        if self.__vision_data_available == False:
            rospy.logerr(
                "[databased_grasp_generation]: get vision data failed!"
            )
            return False

        from neurapy_ai.utils.ros_conversions import (
            geometry_msg_pose_2_transformation_matrix,
        )
        from geometry_msgs.msg import Pose

        camera_link_to_root = (
            Pose()
        )  # [0.864, -0.156, 1.041], [0.688, 0.724, 0.030, -0.037], camera_to_root: [0.099, -0.847, 1.061], [0.688, 0.724, 0.030, 0.037]
        camera_link_to_root.position.x = 0.109  # 0.864
        camera_link_to_root.position.y = -0.767  # -0.156
        camera_link_to_root.position.z = 1.073  # 1.041
        camera_link_to_root.orientation.x = 0.693
        camera_link_to_root.orientation.y = 0.721
        camera_link_to_root.orientation.z = -0.024
        camera_link_to_root.orientation.w = 0.013
        T_camera_link_to_root = geometry_msg_pose_2_transformation_matrix(
            camera_link_to_root
        )

        #####################################################################
        # ************** instance segmentation data **************
        instance_seg_start_time = rospy.Time.now().to_sec()
        rospy.loginfo(
            "[instance segmentation data]: start instance segmentation"
        )

        from neurapy_ai.clients.instance_segmentation_client import (
            InstanceSegmentationClient,
        )

        instance_seg_client = InstanceSegmentationClient(
            model_name="tower_building", model_version="v_6"
        )
        (
            seg_return_code,
            segmented_instances,
            segmentation_mask,
        ) = instance_seg_client.get_segmented_instances_from_image_ros(
            self.rgb_image_data,
            class_names=object_names,
        )

        class_instance_indexes = []
        for seg_instance in segmented_instances:
            # class_instance_indexes.append(seg_instance.class_instance_index)
            # rospy.loginfo("[databased_grasp_generation][instance segmentation data]: segmented class name %s and index: %lu", seg_instance.class_name, seg_instance.class_instance_index)
            class_instance_indexes.append(seg_instance.segmentation_index)
            rospy.loginfo(
                "[databased_grasp_generation][instance segmentation data]: segmented class name %s and index: %lu",
                seg_instance.class_name,
                seg_instance.segmentation_index,
            )

        rospy.logwarn(
            "[databased_grasp_generation][instance segmentation data]: instance segmentation tooks %s sec.",
            str(rospy.Time.now().to_sec() - instance_seg_start_time),
        )
        # confirm = input(
        #     "[databased_grasp_generation][instance segmentation data] confirm execution..."
        # )

        #####################################################################
        # ************** pose estimation data **************
        pose_estimation_start_time = rospy.Time.now().to_sec()
        rospy.loginfo(
            "[databased_grasp_generation][pose estimation data]: start pose estimation."
        )
        from pose_estimation_ros_msgs.msg import DetectedObject
        from neura_ai_robot_api.clients.pose_estimation_client import (
            PoseEstimationClient,
        )

        pose_estimation_client = PoseEstimationClient()
        (
            pose_estimation_return_code,
            detected_poses,
        ) = pose_estimation_client.get_poses_from_segmentation_result_ros(
            color_image=self.rgb_image_data,
            depth_image=self.depth_image_data,
            segmented_instances=segmented_instances,
            segmentation_mask=segmentation_mask,
            camera_intrinsics=self.camera_info_data,
            target_to_camera=T_camera_link_to_root,
            target_frame="root_link",
            include_mask_ids=class_instance_indexes,
            class_names=object_names,
        )
        rospy.logwarn(
            "[databased_grasp_generation][pose estimation data]:: pose_estimation_return_code message: %s and detection result: %lu",
            pose_estimation_return_code.message,
            len(detected_poses),
        )

        for detected_pose in detected_poses:
            rospy.loginfo(
                "[databased_grasp_generation][pose estimation data]:: class name %s and segmentation index %lu",
                detected_pose.class_name,
                detected_pose.segmentation_index,
            )
        rospy.logwarn(
            "[databased_grasp_generation][pose estimation data]: pose estimation tooks %s sec.",
            str(rospy.Time.now().to_sec() - pose_estimation_start_time),
        )

        instance_ids = []
        if len(object_names) == 0:
            for detected_pose in detected_poses:
                object_names.append(detected_pose.class_name)
                instance_ids.append(
                    detected_pose.class_name
                    + "_"
                    + str(detected_pose.segmentation_index)
                )

        print(object_names)

        bin_name = "workspace_2023"
        bin_detection_success, bin_pose, bin_bbox = self.bin_detection(
            bin_name=bin_name, is_known_bin_pose=True
        )
        print(bin_pose)
        print(bin_bbox)
        confirm = input("Confirm")

        #####################################################################
        # ************** Call function with correct parameters **************
        grasp_generator_return_code = grasp_generator_api.generate_grasps_ros(
            object_names=object_names,
            workspace_name="workspace_2023",
            bin_name=bin_name,
            gripper_name="RobotiQ_140",
            raw_point_cloud=self.point_cloud_data,
            raw_rgb_image=self.rgb_image_data,
            raw_depth_image=self.depth_image_data,
            raw_camera_info=self.camera_info_data,
            segmentation_mask=segmentation_mask,
            instance_ids=instance_ids,
            segmented_instances=segmented_instances,
            detected_poses=detected_poses,
            is_known_bin_pose=True,
            bin_pose=bin_pose,
            bin_bbox=bin_bbox,
            check_reachability=False,
            camera_link_to_root=PoseStamped(),
        )

        print(grasp_generator_return_code)

    def bin_detection(
        self,
        bin_name: str = "",
        is_known_bin_pose: bool = False,
    ) -> Tuple[bool, PoseStamped, Vector3]:
        bin_detection_start_time = rospy.Time.now().to_sec()
        rospy.loginfo("[bin_detection]: start bin detection")
        need_bin_detection = True

        bin_pose = PoseStamped()
        bin_bbox = Vector3()
        if is_known_bin_pose and bin_name != "":
            from neurapy_ai.utils.return_codes import ReturnCodes
            from neurapy_ai.clients.database_client import DatabaseClient

            database_client = DatabaseClient()
            database_return_code, bin_workspace = database_client.get_workspace(
                workspace_name=bin_name
            )

            from neurapy_ai.utils.ros_conversions import (
                pose_2_geometry_msg_pose,
            )

            bin_pose.header.frame_id = bin_workspace.frame
            bin_pose.header.stamp = rospy.Time.now()
            bin_pose.pose = pose_2_geometry_msg_pose(bin_workspace.pose)

            bin_bbox.x = bin_workspace.len_x
            bin_bbox.y = bin_workspace.len_y
            bin_bbox.z = bin_workspace.len_z

            # hack: change position in z axis
            bin_pose.pose.position.z = 0.25

            if (
                need_bin_detection
                and database_return_code.value == ReturnCodes.SUCCESS
            ):
                rospy.loginfo(
                    "[bin_detection]: get bin workspace sucessful. message: %s",
                    database_return_code.message,
                )
                from neura_ai_robot_api.clients.bin_detection_client import (
                    BinDetectionClient,
                )

                bin_detection_client = BinDetectionClient()
                (
                    bin_detection_return_code,
                    bin_detection_pose,
                ) = bin_detection_client.get_bin_pose(bin_name=bin_name)
                if bin_detection_return_code.value == ReturnCodes.SUCCESS:
                    rospy.loginfo(
                        "[bin_detection]: get bin workspace sucessful. message: %s",
                        bin_detection_return_code.message,
                    )

                    from neurapy_ai.utils.ros_conversions import (
                        pose_2_geometry_msg_pose,
                    )

                    bin_pose.header.frame_id = bin_workspace.frame
                    bin_pose.header.stamp = rospy.Time.now()
                    bin_pose.pose = pose_2_geometry_msg_pose(
                        bin_detection_pose.pose
                    )
                    bin_bbox.x = bin_workspace.len_x
                    bin_bbox.y = bin_workspace.len_y
                    bin_bbox.z = bin_workspace.len_z
                else:
                    rospy.logerr(
                        "[databased_grasp_generation][bin_detection]: bin detection failed. message: %s",
                        bin_detection_return_code.message,
                    )
                    return False, PoseStamped(), Vector3()
            elif need_bin_detection:
                rospy.logerr(
                    "[databased_grasp_generation][bin_detection]: get bin workspace failed. message: %s",
                    database_return_code.message,
                )
                return False, PoseStamped(), Vector3()

        rospy.logwarn(
            "[bin_detection]: bin detection tooks %s sec.",
            str(rospy.Time.now().to_sec() - bin_detection_start_time),
        )
        return True, bin_pose, bin_bbox

    def _get_vision_data_callback(
        self, point_cloud_msg, rgb_image_msg, depth_image_msg, camera_info_msg
    ):
        self.point_cloud_data = point_cloud_msg
        self.rgb_image_data = rgb_image_msg
        self.depth_image_data = depth_image_msg
        self.camera_info_data = camera_info_msg
        self.__vision_data_available = True


if __name__ == "__main__":
    # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
    rospy.init_node("test_generate_grasp_client_api")

    grasp_generator_api = GraspGeneratorClient()
    rospy.sleep(5.0)

    test_grasp_generator_api = TestGraspGeneratorAPI()
    # test_grasp_generator_api.test_start_detection(
    #    grasp_generator_api=grasp_generator_api, object_names=[]
    # )
    test_grasp_generator_api.test_generate_grasps_ros(
        grasp_generator_api=grasp_generator_api, object_names=[]
    )
