from neurapy_ai.clients.marker_detection_client import MarkerDetectionClient
import rospy

if __name__ == '__main__':
    rospy.init_node('test_marker_detection',
                    anonymous=True)
    detect_marker_client = MarkerDetectionClient()
    # http://jevois.org/moddoc/DemoArUco/screenshot2.png can be used for testing it's 4x4 dictionary
    res, markers = detect_marker_client.get_detected_markers(1)

    for marker in markers:
        print(marker.pose)
        print(marker.id)