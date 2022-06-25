#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, Point
from mascot.msg import Force, Gain
from std_msgs.msg import Bool
import yaml
import signal
from matplotlib import pyplot as plt
import time
import numpy as np


rospy.init_node("Controller", anonymous=True) # initialize ros node

file_path = __file__
dir_path = file_path[: (len(file_path) - len("controller.py"))] # get into the direcory

with open(dir_path + "config.yaml", "r") as yamlfile:
    data = yaml.load(yamlfile, Loader=yaml.FullLoader)


my_gain_d = data["Control"]["Position_params"]["kd"]
my_gain_p = data["Control"]["Position_params"]["kp"]
con_kp = data["Control"]["Consensus_params"]["con_kp"]
con_kd = data["Control"]["Consensus_params"]["con_kd"]
max_force = data["Robot"]["Max_force"]
num_agent = data["Robot"]["Number"]
leader = data["Control"]["Consensus_params"]["Leader"]
Position_flag = data["Control"]["Position_Control"]
Consensus_flag = data["Control"]["Consensus_Control"]
L_mat = data["Control"]["Consensus_params"]["L_mat"]
Comm_Graph_flag = data["Control"]["Consensus_params"]["Communication_Graph"]

set_pos_x = 0
set_pos_y = 0
set_vel_x = 0
set_vel_y = 0


class Robot:
    def __init__(self, index):
        self.force_pub = rospy.Publisher(
            "/r{i}/cmd_force".format(i=index), Force, queue_size=10
        )

        self.consen_ctrl_pub = rospy.Publisher(
            "/r{i}/drone/consenctrl".format(i=index), Bool, queue_size=10
        )

        rospy.Subscriber("/r{i}/drone/gt_vel".format(i=index), Twist, self.cb_vel)
        rospy.Subscriber("/r{i}/drone/gt_pose".format(i=index), Pose, self.cb_pose)
        rospy.Subscriber("/r{i}/drone/gt_acc".format(i=index), Twist, self.cb_acc)
        rospy.Subscriber("/r{i}/drone/loadPID".format(i=index), Gain, self.pid_gain)
        rospy.Subscriber("/r{i}/cmd_position".format(i=index), Point, self.cb_position)

        self.msg = Force()
        self.pose = Pose()
        self.vel = Twist()
        self.acc = Twist()
        self.position_x = []
        self.position_y = []
        self.velocity_x = []
        self.velocity_y = []
        self.force_in_x = []
        self.force_in_y = []
        self.time_taken = []

    def cb_pose(self, data):
        self.pose = data

    def cb_vel(self, data):
        self.vel = data

    def cb_acc(self, data):
        self.acc = data

    def pid_gain(self, data):
        global my_gain_d, my_gain_p
        my_gain_p = data.k_p
        my_gain_d = data.k_d
        print("Loaded new PID gains")

    def cb_position(self, data):
        global set_pos_x, set_pos_y
        set_pos_x = data.x
        set_pos_y = data.y

    def pub_force(self, force_x, force_y, force_z):
        self.msg.force.x = force_x
        self.msg.force.y = force_y
        self.msg.force.z = force_z
        self.force_pub.publish(self.msg)

    def start_consenctrl(self):
        ctrl = Bool(True)
        self.consen_ctrl_pub.publish(ctrl)
        print("Started Control")

    def append_data(
        self,
        force_x,
        force_y,
        pos_x,
        pos_y,
        vel_x,
        vel_y,
        time_done,
    ):
        self.force_in_x.append(force_x)
        self.force_in_y.append(force_y)
        self.position_x.append(pos_x)
        self.position_y.append(pos_y)
        self.velocity_x.append(vel_x)
        self.velocity_y.append(vel_y)
        self.time_taken.append(time_done)

    def update_force(self):
        global max_force
        my_gain_p = 1.2
        my_gain_d = 2.5
        self.start_consenctrl()
        while not rospy.is_shutdown():
            # print("hii")

            force_x = (
                my_gain_p * (set_pos_x - self.pose.position.x)
                - my_gain_d * self.vel.linear.x
            )
            force_y = (
                my_gain_p * (set_pos_y - self.pose.position.y)
                - my_gain_d * self.vel.linear.y
            )

            if force_x >= max_force or force_x <= -max_force:
                if force_x <= 0:
                    force_x = -max_force
                else:
                    force_x = max_force

            if force_y >= max_force or force_y <= -max_force:
                if force_y <= 0:
                    force_y = -max_force
                else:
                    force_y = max_force

            self.pub_force(force_x, force_y, 0)


robot = [Robot(i + 1) for i in range(num_agent)]

def position_control():

    rospy.sleep(0.5)
    for i in range(num_agent):
        robot[i].start_consenctrl()
        rospy.sleep(0.5)
    program_starts = time.time()
    prev_time=0
    while not rospy.is_shutdown():
        # Write your control Algorithms here
        if(time.time() - prev_time >= 0.01):
            for i in range(num_agent):
                prev_time=time.time()
                # set_pos_x = (sum([robot[j].pose.position.x for j in range(num_agent)])-robot[i].pose.position.x)/(num_agent-1)
                # set_pos_y = (sum([robot[j].pose.position.x for j in range(num_agent)])-robot[i].pose.position.x)/(num_agent-1)

                pos_error_x = set_pos_x - robot[i].pose.position.x
                pos_error_y = set_pos_y - robot[i].pose.position.y

                vel_error_x = set_vel_x - robot[i].vel.linear.x
                vel_error_y = set_vel_y - robot[i].vel.linear.y

                force_x = my_gain_p * (pos_error_x) + my_gain_d * (vel_error_x)
                force_y = my_gain_p * (pos_error_y) + my_gain_d * (vel_error_y)

                if force_x >= max_force or force_x <= -max_force:
                    if force_x <= 0:
                        force_x = -max_force
                    else:
                        force_x = max_force

                if force_y >= max_force or force_y <= -max_force:
                    if force_y <= 0:
                        force_y = -max_force
                    else:
                        force_y = max_force

                now = time.time()

                robot[i].append_data(
                    force_x,
                    force_y,
                    robot[i].pose.position.x,
                    robot[i].pose.position.y,
                    robot[i].vel.linear.x,
                    robot[i].vel.linear.y,
                    now - program_starts,
                ) #  Apepnding the data to draw the plot at the end
                robot[i].pub_force(force_x, force_y, 0)

        # print(" ForceX: "+str(force_x)+" ForceY: "+str(force_y))


def consensus_control():  # Distributed Consensus
    rospy.sleep(0.5)

    for i in range(num_agent):
        robot[i].start_consenctrl()
        rospy.sleep(0.5)
    program_starts = time.time()
    prev_time=0
    while not rospy.is_shutdown():
        # Write your control Algorithms here
        if(time.time() - prev_time >= 0.01):
            prev_time=time.time()
            if (Comm_Graph_flag)==0 :
                L = np.ones((num_agent, num_agent))
                L = L * -1
                np.fill_diagonal(L, num_agent - 1)
            else :
                L=np.array(L_mat)

            for i in range(num_agent):

                X = np.array([[robot[i].pose.position.x for i in range(num_agent)]]).T
                Y = np.array([[robot[i].pose.position.y for i in range(num_agent)]]).T

                Ax = -con_kp * L.dot(X)
                Ay = -con_kp * L.dot(Y)

                vel_error_x = set_vel_x - robot[i].vel.linear.x
                vel_error_y = set_vel_y - robot[i].vel.linear.y

                force_x = float(Ax[i]) + con_kd * vel_error_x
                force_y = float(Ay[i]) + con_kd * vel_error_y


                if force_x >= max_force or force_x <= -max_force:
                    if force_x <= 0:
                        force_x = -max_force
                    else:
                        force_x = max_force

                if force_y >= max_force or force_y <= -max_force:
                    if force_y <= 0:
                        force_y = -max_force
                    else:
                        force_y = max_force

                now = time.time()

                robot[i].append_data(
                    force_x,
                    force_y,
                    robot[i].pose.position.x,
                    robot[i].pose.position.y,
                    robot[i].vel.linear.x,
                    robot[i].vel.linear.y,
                    now - program_starts,
                )
                robot[i].pub_force(force_x, force_y, 0)

def mixmax_consensus():
    rospy.sleep(0.5)
    leader = 0
    beta = [1 for i in range(num_agent)]
    beta[0]=0.95

    for i in range(num_agent):
        if i != leader:
            robot[i].start_consenctrl()
            rospy.sleep(0.5)
    program_starts = time.time()
    prev_time=0
    while not rospy.is_shutdown():
        # Write your control Algorithms here
        if(time.time() - prev_time >= 0.1):
            prev_time=time.time()
            # Write your control Algorithms here

            for i in range(num_agent):
                if i != leader:


                    z_pos_x = robot[i].pose.position.x - robot[i-1].pose.position.x
                    z_vel_x = robot[i].vel.linear.x - robot[i-1].vel.linear.x

                    z_pos_y = robot[i].pose.position.y - robot[i-1].pose.position.y
                    z_vel_y = robot[i].vel.linear.y - robot[i-1].vel.linear.y

                    s_z_x = z_pos_x + (z_vel_x*abs(z_vel_x))/(2*(beta[i]-beta[0]))
                    s_z_y = z_pos_y + (z_vel_y*abs(z_vel_y))/(2*(beta[i]-beta[0]))

                    if(s_z_x==0):
                        force_x = beta[i]*np.sign(z_pos_x)
                    else:
                        force_x = -beta[i]*np.sign(s_z_x)

                    if(s_z_y==0):
                        force_y = beta[i]*np.sign(z_pos_y)
                    else:
                        force_y = -beta[i]*np.sign(s_z_y)

                    if force_x >= max_force or force_x <= -max_force:
                        if force_x <= 0:
                            force_x = -max_force
                        else:
                            force_x = max_force

                    if force_y >= max_force or force_y <= -max_force:
                        if force_y <= 0:
                            force_y = -max_force
                        else:
                            force_y = max_force

                     
                    robot[i].pub_force(force_x, force_y, 0)
                now = time.time()
                robot[i].append_data(
                        10,
                        10,
                        robot[i].pose.position.x,
                        robot[i].pose.position.y,
                        robot[i].vel.linear.x,
                        robot[i].vel.linear.y,
                        now - program_starts,
                    )

def leader_consensus_control():
    rospy.sleep(0.5)

    for i in range(num_agent):
        if i != leader:
            robot[i].start_consenctrl()
            rospy.sleep(0.5)
    program_starts = time.time()
    prev_time=0
    while not rospy.is_shutdown():
        # Write your control Algorithms here
        if(time.time() - prev_time >= 0.01):
            prev_time=time.time()
            # Write your control Algorithms here
            if (Comm_Graph_flag)==0 :
                L = np.ones((num_agent, num_agent))
                L = L * -1
                np.fill_diagonal(L, num_agent - 1)
            
            else :
                L=np.array(L_mat)

            for i in range(num_agent):
                
                X = np.array([[robot[i].pose.position.x for i in range(num_agent)]]).T
                Y = np.array([[robot[i].pose.position.y for i in range(num_agent)]]).T

                Ax = -con_kp * L.dot(X)
                Ay = -con_kp * L.dot(Y)
                vel_error_x = set_vel_x - robot[i].vel.linear.x
                vel_error_y = set_vel_y - robot[i].vel.linear.y

                force_x = float(Ax[i]) + con_kd * vel_error_x
                force_y = float(Ay[i]) + con_kd * vel_error_y

                if force_x >= max_force or force_x <= -max_force:
                    if force_x <= 0:
                        force_x = -max_force
                    else:
                        force_x = max_force

                if force_y >= max_force or force_y <= -max_force:
                    if force_y <= 0:
                        force_y = -max_force
                    else:
                        force_y = max_force

                now = time.time()

                robot[i].append_data(
                    force_x,
                    force_y,
                    robot[i].pose.position.x,
                    robot[i].pose.position.y,
                    robot[i].vel.linear.x,
                    robot[i].vel.linear.y,
                    now - program_starts,
                )
                if i != leader:
                    robot[i].pub_force(force_x, force_y, 0)


def handler(signum, frame):

    print("Saving Figures Please Wait Don't press any Key")

    plt.title("Position")
    for i in range(num_agent):
        plt.plot(
            robot[i].position_x,
            robot[i].position_y,
            label="Robot{num}".format(num=i + 1),
        )
    plt.xlabel("X-Poisiton(m)")
    plt.ylabel("Y-Poisiton(m)")
    plt.legend(loc="upper right", ncol=1)
    if(data["Plots"]["Save_plot"]) : plt.savefig(dir_path + "figure/Position")
    if(data["Plots"]["Show_plot"]) : plt.show()
    plt.clf()
    
    if data["Plots"]["Position"]:
        plt.title("Position-X")
        for i in range(num_agent):
            plt.plot(
                robot[i].time_taken,
                robot[i].position_x,
                label="Robot{num} Position".format(num=i + 1),
            )
        plt.xlabel("Time(s)")
        plt.ylabel("Position(m)")
        plt.legend(loc="upper right", ncol=1)
        if(data["Plots"]["Save_plot"]) : plt.savefig(dir_path + "figure/Position-X")
        if(data["Plots"]["Show_plot"]) : plt.show()
        plt.clf()

        plt.title("Position-Y")
        for i in range(num_agent):
            plt.plot(
                robot[i].time_taken,
                robot[i].position_y,
                label="Robot{num} Position".format(num=i + 1),
            )
        plt.xlabel("Time(s)")
        plt.ylabel("Position(m)")
        plt.legend(loc="upper right", ncol=1)
        if(data["Plots"]["Save_plot"]) : plt.savefig(dir_path + "figure/Position-Y")
        if(data["Plots"]["Show_plot"]) : plt.show()
        plt.clf()
    
    if data["Plots"]["Velocity"]:
        plt.title("Velocity-X")
        for i in range(num_agent):
            plt.plot(
                robot[i].time_taken,
                robot[i].velocity_x,
                label="Robot{num} Velocity".format(num=i + 1),
            )
        plt.xlabel("Time(s)")
        plt.ylabel("Velocity(m/s)")
        plt.legend(loc="upper right", ncol=1)
        if(data["Plots"]["Save_plot"]) : plt.savefig(dir_path + "figure/Velocity-X")
        if(data["Plots"]["Show_plot"]) : plt.show()
        plt.clf()

        plt.title("Velocity-Y")
        for i in range(num_agent):
            plt.plot(
                robot[i].time_taken,
                robot[i].velocity_y,
                label="Robot{num} Velocity".format(num=i + 1),
            )
        plt.xlabel("Time(s)")
        plt.ylabel("Velocity(m/s)")
        plt.legend(loc="upper right", ncol=1)
        if(data["Plots"]["Save_plot"]) : plt.savefig(dir_path + "figure/Velocity-Y")
        if(data["Plots"]["Show_plot"]) : plt.show()
        plt.clf()

    exit(1)


signal.signal(signal.SIGINT, handler)


if __name__ == "__main__":
    try:
        if data["Control"]["Position_Control"]:
            position_control()
        elif data["Control"]["Consensus_Control"] :
            if data["Control"]["Consensus_params"]["Leader"]!=0 :
                leader_consensus_control()
            elif data["Control"]["Consensus_params"]["minmax"]:
                mixmax_consensus()
            else:
                consensus_control()
    except KeyboardInterrupt:
        pass
