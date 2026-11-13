import numpy as np
import math

theta1=math.radians(0)
theta2=math.radians(0)
theta3=math.radians(0)
theta4=math.radians(0)
theta5=math.radians(0)
theta6=math.radians(0)
theta7=math.radians(0)

R01=np.array([[math.cos(theta1),0,-math.sin(theta1)],
             [math.sin(theta1),0,math.cos(theta1)],
             [0,-1,0]])

R12=np.array([[math.cos(theta2),0,math.sin(theta2)],
             [math.sin(theta2),0,-math.cos(theta2)],
             [0,-1,0]])

R23=np.array([[math.cos(theta3),0,-math.sin(theta3)],
             [math.sin(theta3),0,math.cos(theta3)],
             [0,-1,0]])

R34=np.array([[math.cos(theta4),0,math.sin(theta4)],
             [math.sin(theta4),0,-math.cos(theta4)],
             [0,-1,0]])

R45=np.array([[math.cos(theta5),0,-math.sin(theta5)],
             [math.sin(theta5),0,math.cos(theta5)],
             [0,-1,0]])

R56=np.array([[math.cos(theta6),0,math.sin(theta6)],
             [math.sin(theta6),0,-math.cos(theta6)],
             [0,-1,0]])

R67=np.array([[math.cos(theta7),-math.sin(theta7),0],
             [math.sin(theta7),math.cos(theta7),0],
             [0,0,1]])

R07=R01@R12@R23@R34@R45@R56@R67
print(R07)


v1=1
v2=1
v3=1
v4=1
v5=1
v6=1
v7=1
h1=1
h2=1
h3=1

T01=np.array([[R01[0, 0],R01[0, 1],R01[0, 2],0]
             ,[R01[1, 0],R01[1, 1],R01[1, 2],0]
             ,[R01[2, 0],R01[2, 1],R01[2, 2],v1]
             ,[0,0,0,1]])

T12=np.array([[R12[0, 0],R12[0, 1],R12[0, 2],0]
             ,[R12[1, 0],R12[1, 1],R12[1, 2],-v2]
             ,[R12[2, 0],R12[2, 1],R12[2, 2],h1]
             ,[0,0,0,1]])

T23=np.array([[R23[0, 0],R23[0, 1],R23[0, 2],0]
             ,[R23[1, 0],R23[1, 1],R23[1, 2],0]
             ,[R23[2, 0],R23[2, 1],R23[2, 2],v3]
             ,[0,0,0,1]])

T34=np.array([[R34[0, 0],R34[0, 1],R34[0, 2],0]
             ,[R34[1, 0],R34[1, 1],R34[1, 2],v4]
             ,[R34[2, 0],R34[2, 1],R34[2, 2],h2]
             ,[0,0,0,1]])

T45=np.array([[R45[0, 0],R45[0, 1],R45[0, 2],0]
             ,[R45[1, 0],R45[1, 1],R45[1, 2],0]
             ,[R45[2, 0],R45[2, 1],R45[2, 2],v5]
             ,[0,0,0,1]])

T56=np.array([[R56[0, 0],R56[0, 1],R56[0, 2],0]
             ,[R56[1, 0],R56[1, 1],R56[1, 2],v6]
             ,[R56[2, 0],R56[2, 1],R56[2, 2],h3]
             ,[0,0,0,1]])

T67=np.array([[R67[0, 0],R67[0, 1],R67[0, 2],0]
             ,[R67[1, 0],R67[1, 1],R67[1, 2],0]
             ,[R67[2, 0],R67[2, 1],R67[2, 2],v7]
             ,[0,0,0,1]])


T02=T01@T12
T03=T01@T12@T23
T04=T01@T12@T23@T34
T05=T01@T12@T23@T34@T45
T06=T01@T12@T23@T34@T45@T56
T07=T01@T12@T23@T34@T45@T56@T67

T_mat_list=[T01,T02,T03,T04,T05,T06,T07]
J_mat=np.zeros([6,7])
x=1
y=1
z=1
p_cil=[x,y,z]
q=[]
J_cil=np.array([[10],
                [15],
                [10],
                [0],
                [1],
                [0]])

for i in range(0,len(T_mat_list)):
    
    skalar=np.cross(T_mat_list[i][:3,2],(p_cil-T_mat_list[i][:3,3]))
    vektor=T_mat_list[i][:3,2]
    J=np.array([skalar[0],skalar[1],skalar[2],vektor[0],vektor[1],vektor[2]])
    J_mat[:,i]=J

q=np.linalg.pinv(J_mat) @ J_cil
print(np.degrees(q))
