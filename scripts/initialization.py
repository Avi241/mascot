#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool,Empty
from geometry_msgs.msg import Point
import yaml

file_path = __file__
dir_path = file_path[: (len(file_path) - len("initialization.py"))]

with open(dir_path + "config.yaml", "r") as yamlfile:
    data = yaml.load(yamlfile, Loader=yaml.FullLoader)  # Get yaml file data

num_agent = data["Robot"]["Number"]

if data["Robot"]["InitialPose"] is False :
    with open(dir_path + "temp.yaml", "r") as yamlfile:
        data = yaml.load(yamlfile, Loader=yaml.FullLoader)

rospy.init_node("Initialization", anonymous=True) # Initialize ros node


def takeoff(num_agent):
    pub = [
        rospy.Publisher("/r{index}/drone/takeoff".format(index=i + 1), Empty, queue_size=10)
        for i in range(num_agent)
    ] # Create a takeoff publisher for each drone
    rospy.sleep(0.5) # delay for ros to initalize and create a publisher otherwise it won't be able to accept the published command
    for p in pub:
        p.publish(Empty()) # Publish empty msg to takeoff

def position(num_agent,data):
    pub = [
        rospy.Publisher("/r{index}/cmd_position".format(index=i + 1), Point, queue_size=10)
        for i in range(num_agent)
    ]
    rospy.sleep(0.5)
    for i,p in enumerate(pub):
        pos_msg = Point()
        pos_msg.x = data["Robot"]["Position"]["r{r_n}".format(r_n=i+1)]["x"] 
        pos_msg.y = data["Robot"]["Position"]["r{r_n}".format(r_n=i+1)]["y"]
        pos_msg.z = data["Robot"]["Position"]["r{r_n}".format(r_n=i+1)]["z"]
        p.publish(pos_msg) # Publish position

def posctrl(num_agent):
    pub = [
        rospy.Publisher("/r{index}/drone/posctrl".format(index=i + 1), Bool, queue_size=10)
        for i in range(num_agent)
    ]
    rospy.sleep(0.5)
    for p in pub:
        pos_msg = Bool(True)
        p.publish(pos_msg) # Enable position control

if __name__ == "__main__":
    try:
        takeoff(num_agent)
        position(num_agent,data)
        posctrl(num_agent)
    except rospy.ROSInterruptException:
        pass
