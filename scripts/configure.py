#!/usr/bin/env python3

import os
import yaml
import random
import numpy as np

def write_yaml(data):
    """ A function to write YAML file"""
    with open('temp.yaml', 'w') as f:
        yaml.dump(data, f)

file_path = __file__ # curremnt file path
dir_path = file_path[: (len(file_path) - len("scripts/configure.py"))]

with open(dir_path + "scripts/config.yaml", "r") as yamlfile:
    data = yaml.load(yamlfile, Loader=yaml.FullLoader)

file = open(dir_path + "launch/robots.launch", "w+") 
file.truncate(0)  # Delete everthing in FILE

agent_numb = data["Robot"]["Number"]  ######   for agent_number

if data["Robot"]["InitialPose"] is False :
    li=range(-agent_numb*2,agent_numb*2) ## generate num between -n to +n
    pos_arr = np.array(random.sample(li,agent_numb*2)) # Sample without repetattion
    # Convert 1D array to a 2D numpy array of 2 rows and 3 columns
    arr_2d = np.reshape(pos_arr, (agent_numb, 2))
    dicts = {}
    keys = ['x','y','z']
    for j in range(agent_numb):
        temp_dic = {}
        temp_dic['x'] = int(arr_2d[j][0])
        temp_dic['y'] = int(arr_2d[j][1])
        temp_dic['z'] = j+1
        dicts['r{num}'.format(num=j+1)]=temp_dic
    write_yaml({'Robot': {'Number':agent_numb,'Position':dicts}})

    with open(dir_path + "scripts/temp.yaml", "r") as yamlfile:
        data = yaml.load(yamlfile, Loader=yaml.FullLoader)


file.write("<launch>") # start writing launch file

for x in range(int(agent_numb)):

    num = x + 1
    file.write(
        """

            <group ns="r{no}">
                <arg name="ns" value="r{no}"/>
                <param name="robot_description" command="$(find xacro)/xacro $(find mascot)/urdf/model.xacro robot_name:=$(arg ns)" />                                        
                <include file="$(find mascot)/launch/one_robot.launch">
                <arg name="init_pose" value="-x {posx} -y {posy} -z 0" />
                <arg name="robot_name" value="r{no}" />
                </include>
            </group>

            """.format(
            no=num,
            posx=data["Robot"]["Position"]["r{r_n}".format(r_n=num)]["x"],
            posy=data["Robot"]["Position"]["r{r_n}".format(r_n=num)]["y"],
        )
    )

file.write("</launch>")
file.close() # close launch file

dir_path = file_path[: (len(file_path) - len("scripts/configure.py"))]
file = open(dir_path + "launch/controller.launch", "w+") 
file.truncate(0)

file.write("<launch>")

for x in range(int(agent_numb)):

    if data["Control"]["Custom_Control_Law"]:
        num = x + 1
        file.write(
            """
                <node name="control{num}" pkg="mascot" type="control.py" output="screen" args="-index {num}" /> 
            """.format(
                num=num)
        )

    if data["Control"]["Position_Control"]:
        num = x + 1
        file.write(
            """
                <node name="control{num}" pkg="mascot" type="control.py" output="screen" args="-index {num}" /> 
            """.format(
                num=num)
        )
    elif data["Control"]["Consensus_Control"] :
        if data["Control"]["Consensus_params"]["Leader"]!=0 :
            leader = data["Control"]["Consensus_params"]["Leader"]
            if x!=leader - 1  :
                num = x + 1
                file.write(
                    """
                        <node name="control{num}" pkg="mascot" type="control.py" output="screen" args="-index {num}" /> 
                    """.format(
                        num=num)
                )
        elif data["Control"]["Consensus_params"]["minmax"]:
            leader = 0
            if x!=leader :
                num = x + 1
                file.write(
                    """
                        <node name="control{num}" pkg="mascot" type="control.py" output="screen" args="-index {num}" /> 
                    """.format(
                        num=num)
                )
        else:
            num = x + 1
            file.write(
                """
                    <node name="control{num}" pkg="mascot" type="control.py" output="screen" args="-index {num}" /> 
                """.format(
                    num=num)
            )

file.write("</launch>")
file.close()
print("Configuration Done")
