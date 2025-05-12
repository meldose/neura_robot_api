import rospy
import math

from neura_ai_robot_api.clients.motion_planning_client import MotionPlanningClient
from neura_ai_robot_api.utils.types import SolidPrimitive
from neurapy_ai.utils.types import Pose, JointState
from sensor_msgs.msg import Image, PointCloud2


if __name__ == "__main__" :
    rospy.init_node("test_motion_planning")
    PSC = MotionPlanningClient()

    param = PSC.PlannerParam()
    param.wait_for_user_confirm = True

    PSC.update_planning_params(param)
    
    goal_pose = Pose([1.0,0.0,1.0],[-math.pi,0.0,-math.pi])
    goal_pose2 = Pose([1.0,0.0,0.5],[-math.pi,0.0,-math.pi])
    joint_pose=JointState([100*math.pi/180,25*math.pi/180,160*math.pi/180,-75*math.pi/180,9*math.pi/180,-78*math.pi/180,-96*math.pi/180])

    joint_pose1=[103*math.pi/180,25*math.pi/180,160*math.pi/180,-75*math.pi/180,9*math.pi/180,-78*math.pi/180,-96*math.pi/180]
    joint_pose2=[24*math.pi/180,33*math.pi/180,-11*math.pi/180,48*math.pi/180,-1*math.pi/180,99*math.pi/180,6*math.pi/180]
    joint_poses = [joint_pose1, joint_pose2]

    states = PSC.get_current_joint_states()

    ReturnCode, Trajectory = PSC.plan_motion_to_joint_goal(joint_pose)
    ReturnCode, Trajectory = PSC.plan_motion_to_pose_goal(goal_pose)
    ReturnCode, Trajectory = PSC.plan_motion_via_cartesian_path([goal_pose2])
