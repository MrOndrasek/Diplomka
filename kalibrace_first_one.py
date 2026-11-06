import math
import numpy as np
import h5py

with h5py.File("") as f


def leika_measurement_to_coordinates(distance,angle_Hz,angle_Vt):
    distance_in_mm=distance*1000
    x=distance_in_mm*math.sin(angle_Vt)*math.cos(angle_Hz)
    y=distance_in_mm*math.sin(angle_Vt)*math.sin(angle_Hz)
    z=distance_in_mm*math.sin(angle_Vt)

    return x,y,z

def forward_kin_xyz(q1,q2,q3,q4,q5,q6,q7):
    #rovnice pro model dopredne kinematiky
    return x,y,z

def transformation_matrix(T1,T2,T3):
    rT2=T2-T1
    rT3=T3-T1
    iT=(rT2-T1)/np.linalg.norm(rT2-T1)
    kT=(rT2)@(rT3)/np.linalg.norm((rT2)@(rT3))
    jT=(kT@iT)/np.linalg.norm(kT@iT)
    R=[iT,kT,jT]
    d=T1
    matrix=np.array([[R[0],d[0]],
                     [R[1],d[1]],
                     [R[2],d[2]],
                     [0,1]])

    return matrix

def robot_base():
    


