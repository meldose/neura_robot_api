from neura_ai_robot_api.clients.bin_detection_client import BinDetectionClient
from neurapy_ai.utils.return_codes import ReturnCodes
from neura_bin_detection_msgs.msg import MethodType
import rospy

if __name__ == "__main__":
    rospy.init_node("test_bin_detection_client_node")
    bdc_ = BinDetectionClient()
    rc, obj_w_pose = bdc_.detect_bin_pose(bin_name="bin_1_bin_pick_box", workspace_name="workspace_1", method=MethodType.WITH_PREPROCESSING)
    if(rc.value!=ReturnCodes.SUCCESS):
        print(rc.message)
    else:
        print(f"Bin {obj_w_pose.name} has a Pose: {obj_w_pose.pose}")
