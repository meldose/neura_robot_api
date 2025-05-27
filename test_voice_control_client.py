#!/usr/bin/env python3
"""
Test the voice command client
"""

from neurapy_ai.clients.voice_control_client import VoiceControlClient
import rospy #imported rospy
import time #imported time module


# calling the main function
 
if __name__ == "__main__":
    rospy.init_node("test_voice_control_clinet") # initiliase the node 
    _voice_control_client = VoiceControlClient()# starting the voice control client 
    _voice_control_client.start_continous_detection()# starting the voice control client for continous detection
    _voice_control_client.get_command_with_trigger("move joint")# starting the voice control client with command trigger
    while True:
        res, command = _voice_control_client.get_last_command()
        # if command != "" and command != "reset":
        print(command)
        time.sleep(1)
    # print(voice_control_client.get_command_with_trigger(command))
    # print(voice_control_client.get_value())
