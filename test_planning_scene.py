import rospy #imported rospy module
import rospkg # imported rospkg
from pathlib import Path # imported the path
from neura_ai_robot_api.clients.planning_scene_client import PlanningSceneClient
from neura_ai_robot_api.utils.types import SolidPrimitive
from neurapy_ai.utils.types import Pose
from sensor_msgs.msg import PointCloud2, Imag

# calling the main function

if __name__ == "__main__" :
    rospy.init_node("test_planning_scene")
    PSC = PlanningSceneClient()
    box1 = SolidPrimitive.Box([1,1,1], Pose.from_list([0, 0, 2, 0, 0, 0]), "root_link")
    box2 = SolidPrimitive.Box([0.5, 0.5,1], Pose.from_list([0, 0, 2.5, 0, 0, 0]), "root_link")
    box3 = SolidPrimitive.Box([0.1,0.1,0.1], Pose.from_list([0.5, 0, 0.5, 0, 0, 0]), "root_link")
    sphere1 = SolidPrimitive.Sphere(0.5,Pose.from_list([1.0, 0, 2, 0, 0, 0]), "root_link")

    PSC.add_boxes([box1,box2], "test")
    PSC.add_spheres([box1,box2], "test1")
    PSC.attach_spheres([sphere1], "test2")
    PSC.attach_boxes([box3], "test3")
    PSC.detach_solid([box3, sphere1],"test4")

    rospack = rospkg.RosPack()
    package_path = rospack.get_path("neura_ai_motion_planner")
    path_to_mesh = package_path + "/scene_creator/test/resources/koalacandy.stl"

    PSC.add_mesh([path_to_mesh],[Pose.from_list([1.0, 1.0, 0, 0, 0, 0])],"test5")
    PSC.remove([path_to_mesh],[Pose.from_list([1.0, 1.0, 0, 0, 0, 0])],"test5")

    PSC.attach_mesh([path_to_mesh],[Pose.from_list([1.0, 0, 0, 0, 0, 0])],"test5")
    PSC.detach_mesh([path_to_mesh],[Pose.from_list([1.0, 0, 0, 0, 0, 0])],"test6")

    depth_msg = rospy.wait_for_message(
                "/camera/aligned_depth_to_color/image_raw", Image, 3
            )
    PSC.publish_octomap_from_depth_image(depth_msg)
    PSC.publish_octomap_from_topic("/camera/color/image_raw")
    PSC.clear_octomap()
    pcl = rospy.wait_for_message(
            "/camera/depth_registered/points", PointCloud2, timeout=5
        )
    PSC.publish_octomap_from_pcd(pcl)
    PSC.publish_octomap_from_depth_image_topic("/camera/aligned_depth_to_color/image_raw")
