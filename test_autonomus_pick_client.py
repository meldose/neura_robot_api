from neurapy_ai.clients.autonomous_pick_client import (
    AutonomousPickClient,
)
import rospy # imported rospy modules

# calling the main function
if __name__ == "__main__":
    rospy.init_node("test_autonomous_pick_client_node")

    print("### test grasp detection with random pick ###")
    workspace_name = "test_table"
    bin_name = ""
    gripper_name = "RobotiQ"
    object_names = []
    AP = AutonomousPickClient()
    return_code = AP.start_detection(
        object_names, workspace_name, bin_name, gripper_name
    )
    return_code, picks = AP.get_picks()
    print(f"Get return code: {return_code.value} , {return_code.message}")
    for pick in picks:
        print(
            f"Get pick: \n  grasp_id: {pick.grasp_id},\n  pre pose: "
            + f"{pick.approach_sequence.pre_pose.__dict__},\n  pose: "
            + f"{pick.approach_sequence.pose.__dict__}, \n  post pose: "
            + f"{pick.approach_sequence.post_pose.__dict__}"
        )

    print("### test grasp detection with random pick ###")
    AP = AutonomousPickClient()
    return_code = AP.start_detection(
        object_names, workspace_name, bin_name, gripper_name
    )
    return_code, picks = AP.get_picks()
    print(f"Get return code: {return_code.value} , {return_code.message}")
    for pick in picks:
        print(
            f"Get pick: \n  grasp_id: {pick.grasp_id},\n  pre pose: "
            + f"{pick.approach_sequence.pre_pose.__dict__},\n  pose: "
            + f"{pick.approach_sequence.pose.__dict__}, \n  post pose: "
            + f"{pick.approach_sequence.post_pose.__dict__}"
        )
