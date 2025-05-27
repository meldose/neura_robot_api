#!/usr/bin/env python3
"""
Test the instance segmentation client
"""

from neurapy_ai.clients import InstanceSegmentationClient
import rospkg # imported rospkg
import rospy # imported rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image # imported Image
from pathlib import Path
import yaml # imported yaml
import cv2 #imoprted cv2 module
import numpy as np # imported numpy


# created function for displaying the result

def display_result(instances, visualization, return_code, ground_truth):
    print("return code: ", return_code.value, return_code.message)
    print("number of detections: ", len(instances))

    assign, ious = _assign_ground_truth(instances, ground_truth)

    header = "        class        |     assigned class     |   iou   "
    print(header)
    print("-" * len(header))
    for ind, iou in enumerate(ious):
        class_str = "{:^19}".format(instances[ind].class_name)
        ind_string = "{:^22}".format(
            ground_truth[assign[ind]]["name"] + "_" + str(assign[ind])
        )
        iou_string = "{:^9}".format(str(np.round(iou, 3)))
        print(f" {class_str} | {ind_string} | {iou_string}")

    cv2.imshow("vis", visualization) # getting the visualisation
    cv2.waitKey(0) # close the window by pressing the key

# created function for assigning the groung truth 

def _assign_ground_truth(predicted, ground_truth):
    """
    Try to assign the best fitting ground truth annotation to each detected
    instance
    """
    assign_indices = -1 * np.ones((len(predicted)), dtype=np.uint8)
    ious = np.zeros((len(predicted)), dtype=np.float32)

    for det_ind, instance in enumerate(predicted):
        class_name = instance.class_name # defining the class name 
        max_iou = 0

        bb = [
            instance.bounding_box.min_x,
            instance.bounding_box.min_y,
            instance.bounding_box.max_x,
            instance.bounding_box.max_y,
        ]

        for ind, gt in enumerate(ground_truth):
            if class_name != gt["name"]:
                continue

            # calculate iou
            bb_gt = [
                gt["bb"][0],
                gt["bb"][1],
                gt["bb"][0] + gt["bb"][2],
                gt["bb"][1] + gt["bb"][3],
            ]
            iou = _get_iou(bb, bb_gt)
            if iou > max_iou:
                assign_indices[det_ind] = ind
                ious[det_ind] = iou
                max_iou = iou 

    return assign_indices, ious

# function for getting the iou

def _get_iou(bb1, bb2):
    """
    Calculate the Intersection over Union (IoU) of two bounding boxes.

    Parameters
    ----------
    bb1 : list ['x1', 'x2', 'y1', 'y2']
    bb1 : list ['x1', 'x2', 'y1', 'y2']

    Returns
    -------
    iou : float
        Intersection over union score in [0, 1]
    """
    # determine the coordinates of the intersection rectangle
    x_left = max(bb1[0], bb2[0]) 
    y_top = max(bb1[1], bb2[1])
    x_right = min(bb1[2], bb2[2])
    y_bottom = min(bb1[3], bb2[3])

    if x_right < x_left or y_bottom < y_top:
        return 0.0

    # The intersection of two axis-aligned bounding boxes is always an
    # axis-aligned bounding box
    intersection_area = (x_right - x_left) * (y_bottom - y_top)

    # compute the area of both AABBs
    bb1_area = (bb1[2] - bb1[0]) * (bb1[3] - bb1[1])
    bb2_area = (bb2[2] - bb2[0]) * (bb2[3] - bb2[1])

    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = intersection_area / float(bb1_area + bb2_area - intersection_area)
    assert iou >= 0.0
    assert iou <= 1.0
    return iou

# created class TestImagePublisher

class TestImagePublisher:

    def __init__(self, image):

        bridge = CvBridge() # created bridge
        self.image_msg = bridge.cv2_to_imgmsg(image, encoding="passthrough") 

        self.im_pub = rospy.Publisher(
            "/camera/color/image_raw", Image, queue_size=1
        ) # creating an publisher 
        rospy.Timer(rospy.Duration(1.0), self._publish_img)
        
# publising the image msg
    def publish_img(self, event):
        self.im_pub.publish(self.image_msg)

# calling the main function

if __name__=="__main__":

    rospack=rospkg.RosPack() # setting up the rospackages

    data_path=Path(rospack.get_path("neura_segmentation"))/"test" # setting up the data path
    test_image_path=data_path/"test_image.jpg" # settign up the image path
    ground_truth_path=data_path/"ground_truth.yaml" # setting up the ground_path
    image=cv2.imread(test_image_path,"r") # getting the image
    with open(ground_truth_path,"r") as gt_file:
        ground_truth=yaml.load(gt_file, Loader=yaml.Loader)


    rospy.init_node("test_instance_segmentaion_client",anonymous=True) # initialise the node 

    client =InstanceSegmentationClient() # setting up the client 
    client.set_model("hrg_industrial") # setting up the client model

    publisher=TestImagePublisher(image) # creating the publishre wiht the image 

    print("testing with image from topic ")

    return_code, instances,mask,input_image=client.get_segmented_instances()
    visualization=client.visualize_segmentatio_result(input_image,instances,mask) # visualization with the result 

    display_result(instances,visualization, return_code, ground_truth) # display the result 

    client.set_parameter("detection_threshold",0.5) # setting up the client 

    print("test with the input image")
    (return_code,instances,mask)=client.get_segmented_instances_from_image(image)

    visualization=client.visualize_segemntation_result(image,instances,mask) # creating the visualization
    display_result(instances,visualization,return_code,ground_truth) # disaply the result 

