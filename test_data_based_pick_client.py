from neurapy_ai.clients.data_based_pick_client import (
    DataBasedPickClient,
)
from neurapy_ai.utils.types import ObjectWithPose
from neurapy_ai.utils.types import Pose
import rospy # imported rospy module

# calling the main function
if __name__ == "__main__":
    rospy.init_node("test_data_based_pick_client_node")
    workspace_name = "test_table"
    bin_name = ""
    gripper_name = "RobotiQ"
    object_names = []

    DP = DataBasedPickClient()
    print("### test grasp detection with known pose ###")
    pose = ObjectWithPose(
        "puzzle_trapezoid",
        Pose(
            [0.9253371149304181, 0.1609944232617458, 0.23948565036486213],
            [
                0.5193110408863679,
                0.4573045936607871,
                -0.5459190446555137,
                -0.47239917081689103,
            ],
        ),
    )
    DP.start_detection_with_known_pose(pose, workspace_name, gripper_name)
    return_code, picks = DP.get_picks()
    print(f"Get return code: {return_code.value} , {return_code.message}")
    for pick in picks:
        print(
            f"Get pick: \n  grasp_id: {pick.grasp_id},\n  pre pose: "
            + f"{pick.approach_sequence.pre_pose.__dict__},\n  pose: "
            + f"{pick.approach_sequence.pose.__dict__}, \n  post pose: "
            + f"{pick.approach_sequence.post_pose.__dict__}"
        )

    print("### test grasp detection ###")
    return_code = DP.start_detection(
        object_names, workspace_name, bin_name, gripper_name
    )
    return_code, picks = DP.get_picks()
    print(f"Get return code: {return_code.value} , {return_code.message}")
    for pick in picks:
        print(
            f"Get pick: \n  grasp_id: {pick.grasp_id},\n  pre pose: "
            + f"{pick.approach_sequence.pre_pose.__dict__},\n  pose: "
            + f"{pick.approach_sequence.pose.__dict__}, \n  post pose: "
            + f"{pick.approach_sequence.post_pose.__dict__}"
        )
