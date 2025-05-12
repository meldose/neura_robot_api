import rospy
import math
from os import getenv

from neura_ai_robot_api.clients.motion_planning_client import MotionPlanningClient
from neurapy_ai.utils.types import Pose, JointState
from neura_ai_robot_api.utils import file_manager
from pathlib import Path



if __name__ == "__main__":

    rospy.init_node("test_file_manager")

    PSC = MotionPlanningClient()

    joint_pose=JointState([0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180,9*math.pi/180,0*math.pi/180,0*math.pi/180])

    ReturnCode, trajectory = PSC.plan_motion_to_joint_goal(joint_pose)

    parent_data_path = str(Path(Path("/home/" + getenv("USER")) / "data")) + "/"

    file_manager.save_trajectory_to_file([trajectory], "trajectory.json")

    ReturnCode, trajectory1 = file_manager.load_trajectory_from_file(parent_data_path + "motion_planning/joint_trajectories/trajectory.json")

    traJ_list, traj_time_list = trajectory.trajectory_to_list()
    print(trajectory.get_end_trajectory_point())
    print(trajectory.get_end_tcp_path_point())
    print(trajectory.get_start_tcp_path_point())
    print(trajectory.get_start_trajectory_point())
    print(trajectory.is_tcp_path_empty())
    print(trajectory.is_empty())

    print(trajectory.get_trajectory_point(1))



    