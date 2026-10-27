import argparse
import importlib
import math
import os
import sys
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence, Type
from python_client.iiwaPy3 import iiwaPy3
import time

ROBOT_IP = "172.31.1.147"

robot = iiwaPy3(ROBOT_IP)

try:
    print("Čtu klouby...")
    joints = robot.getJointsPos()
    print("Klouby:", repr(joints))
    print("Počet hodnot:", len(joints))
    velocity=[0.5]
    souradnice=[[163.77043441322368,-62.68257800110235,-82.39076755465007,-62.599100483568215,41.77663024118516,-61.72226459742321],
    [147.03435507618428,-83.52335408312632,-85.81619047139715,-118.84071210010443,3.888324950130077,-82.29424846296355],
    [158.07475034538135,-61.96149840508223,-79.28584417635906,-86.32195535631809,22.920591458047646,-60.14011900729347],
    [130.19073560587114,-65.20622471264225,-63.600114911118546,-110.933232220799,22.973532365393734,-54.40475591920995],
    [169.02572018906767,-76.6038379943556,-87.42772453718645,-74.42868619556849,26.243189463267655,-76.36346376388859],
    [139.38338302528987,-66.44733284699585,-71.0836878277729,-99.40185402281088,27.23650883270879,-60.13329979342144]]

    for i in range(len(souradnice)):
        joints[0]=math.radians(souradnice[i][0])
        joints[1]=math.radians(souradnice[i][1])
        joints[2]=math.radians(souradnice[i][2]) 
        joints[3]=math.radians(souradnice[i][3])
        joints[4]=math.radians(0)
        joints[5]=math.radians(souradnice[i][4])
        joints[6]=math.radians(souradnice[i][5])
        robot.movePTPJointSpace(joints,velocity)
        print(i)

        joints = robot.getJointsPos()
        for i in range(7):
                temp = joints[i] * 180 / math.pi
                print(temp)
        
        time.sleep(1)


    print("\nČtu TCP...")
    tcp = robot.getEEFPos()
    print("TCP:", repr(tcp))
    print("Počet hodnot:", len(tcp))
    robot.close()

except:
    print("chyba")
    robot.close()
