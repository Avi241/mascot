#!/usr/bin/env python3
from tkinter import *
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import yaml

rospy.init_node("Leader_Controller", anonymous=True)

file_path = __file__
dir_path = file_path[: (len(file_path) - len("leader_control.py"))]

with open(dir_path + "config.yaml", "r") as yamlfile:
    data = yaml.load(yamlfile, Loader=yaml.FullLoader)


if data["Control"]["Consensus_params"]["Leader"]!=0 :
    leader = data["Control"]["Consensus_params"]["Leader"]
elif data["Control"]["Consensus_params"]["minmax"]:
    leader = 1
else:
    leader = 1

pos_pub = rospy.Publisher("/r{num}/cmd_position".format(num=leader), Point, queue_size=10)

pos = Point()
 
root = Tk()
root.geometry("300x500")
root.title(" Gain Publisher ")
 
def Take_input():
    INPUT1 = inputtxt1.get("1.0", "end-1c")
    INPUT2 = inputtxt2.get("1.0", "end-1c")
    INPUT3 = inputtxt3.get("1.0", "end-1c")
    pos.x=float(INPUT1)
    pos.y=float(INPUT2)
    pos.z=float(INPUT3)
    pos_pub.publish(pos)
    Output.insert(END, 'Published')

    

l = Label(text = "X     Y     Z")
inputtxt1 = Text(root, height = 5,
                width = 10,
                bg = "light yellow")

inputtxt2 = Text(root, height = 5,
                width = 10,
                bg = "light yellow")

inputtxt3 = Text(root, height = 5,
                width = 10,
                bg = "light yellow")
 
Output = Text(root, height = 5,
              width = 25,
              bg = "light cyan")
 
Display = Button(root, height = 2,
                 width = 20,
                 text ="Publish",
                 command = lambda:Take_input())
 
l.pack()
inputtxt1.pack()
inputtxt2.pack()
inputtxt3.pack()
Display.pack()
Output.pack()
 
mainloop()
