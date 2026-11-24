import numpy as np
import math


def forward_kinematics(q): #q je list o delce 7 s natocenim kloubu ve stupnich, vyhodi polohu koncoveho efektoru a seznam matic pro IK
    theta1=q[0,0]
    theta2=-q[1,0]
    theta3=-q[2,0]
    theta4=q[3,0]
    theta5=q[4,0]
    theta6=-q[5,0]
    theta7=-q[6,0]
    #rotační matice
    R01=np.array([[math.cos(theta1),0,math.sin(theta1)],
             [math.sin(theta1),0,-math.cos(theta1)],
             [0,1,0]])

    R12=np.array([[math.cos(theta2),0,math.sin(theta2)],
             [math.sin(theta2),0,-math.cos(theta2)],
             [0,1,0]])

    R23=np.array([[math.cos(theta3),0,math.sin(theta3)],
             [math.sin(theta3),0,-math.cos(theta3)],
             [0,1,0]])

    R34=np.array([[math.cos(theta4),0,math.sin(theta4)],
             [math.sin(theta4),0,-math.cos(theta4)],
             [0,1,0]])

    R45=np.array([[math.cos(theta5),0,math.sin(theta5)],
             [math.sin(theta5),0,-math.cos(theta5)],
             [0,1,0]])
    
    R56=np.array([[math.cos(theta6),0,math.sin(theta6)],
             [math.sin(theta6),0,-math.cos(theta6)],
             [0,1,0]])

    R67=np.array([[math.cos(theta7),math.sin(theta7),0],
                [math.sin(theta7),-math.cos(theta7),0],
                [0,0,-1]])

    
    R07=R01@R12@R23@R34@R45@R56@R67
    #print(R07,"\n" ,R01,"\n",R12,"\n",R23,"\n",R34,"\n",R45,"\n",R56,"\n",R67)
    """    print( np.linalg.det(R01))
    print( np.linalg.det(R12))
    print( np.linalg.det(R23))
    print( np.linalg.det(R34))
    print( np.linalg.det(R45))
    print( np.linalg.det(R56))"""
    #rozmery mechanismu(pozdeji upravit)
    v1=360
    v2=0
    v3=420
    v4=0
    v5=400
    v6=0
    v7=126+51
    h1=0
    h2=0
    h3=0

    #transformační matice
    T01=np.array([[R01[0, 0],R01[0, 1],R01[0, 2],0]
             ,[R01[1, 0],R01[1, 1],R01[1, 2],0]
             ,[R01[2, 0],R01[2, 1],R01[2, 2],v1]
             ,[0,0,0,1]])

    T12=np.array([[R12[0, 0],R12[0, 1],R12[0, 2],v2*math.cos(theta2)]
                ,[R12[1, 0],R12[1, 1],R12[1, 2],-v2*math.sin(theta2)]
                ,[R12[2, 0],R12[2, 1],R12[2, 2],v2]
                ,[0,0,0,1]])

    T23=np.array([[R23[0, 0],R23[0, 1],R23[0, 2],0]
                ,[R23[1, 0],R23[1, 1],R23[1, 2],0]
                ,[R23[2, 0],R23[2, 1],R23[2, 2],-v3]
                ,[0,0,0,1]])

    T34=np.array([[R34[0, 0],R34[0, 1],R34[0, 2],v4*math.cos(theta4)]
                ,[R34[1, 0],R34[1, 1],R34[1, 2],-v4*math.sin(theta4)]
                ,[R34[2, 0],R34[2, 1],R34[2, 2],v4]
                ,[0,0,0,1]])

    T45=np.array([[R45[0, 0],R45[0, 1],R45[0, 2],0]
                ,[R45[1, 0],R45[1, 1],R45[1, 2],0]
                ,[R45[2, 0],R45[2, 1],R45[2, 2],v5]
                ,[0,0,0,1]])

    T56=np.array([[R56[0, 0],R56[0, 1],R56[0, 2],v6*math.cos(theta6)]
                ,[R56[1, 0],R56[1, 1],R56[1, 2],-v6*math.sin(theta6)]
                ,[R56[2, 0],R56[2, 1],R56[2, 2],v6]
                ,[0,0,0,1]])
    
    T67=np.array([[R67[0, 0],R67[0, 1],R67[0, 2],0]
                ,[R67[1, 0],R67[1, 1],R67[1, 2],0]
                ,[R67[2, 0],R67[2, 1],R67[2, 2],-v7]
                ,[0,0,0,1]])

    #print("T01 =\n", T01)
    #print("T12 =\n", T12)
    T02=T01@T12
    #print("T02 =\n", T02)
    T03=T01@T12@T23
    T04=T01@T12@T23@T34
    T05=T01@T12@T23@T34@T45
    T06=T01@T12@T23@T34@T45@T56
    T07=T01@T12@T23@T34@T45@T56@T67

    #vrací list matic, T07 v sobě má polohu koncového efektoru
    T00=np.eye(4)
    T_mat_list=[T00,T01,T02,T03,T04,T05,T06,T07]
    p_E=T_mat_list[7][:3,3]
    return p_E,T07



def forward_kinematics_new(q): #q je list o delce 7 s natocenim kloubu ve stupnich, vyhodi polohu koncoveho efektoru a seznam matic pro IK
    
    def Rz(theta):
        c=np.cos(theta)
        s=np.sin(theta)

        R=np.array([
            [c,-s,0],
            [s,c,0],
            [0,0,1]
        ])
        return R

    def Rx(alpha):
        c=np.cos(alpha)
        s=np.sin(alpha)

        R=np.array([
            [1,0,0],
            [0,c,-s],
            [0,s,c]
        ])
        return R


    def transformation(theta,alpha,d):
        T=np.eye(4)

        T[:3,:3]=Rz(theta)@Rx(alpha)
        T[:3,3]=[0,0,d]
        return T


    v1=360
    v3=420
    v5=400
    v7=126+51

    T01=transformation(q[0,0],-np.pi/2,v1)
    T12=transformation(q[1,0],np.pi/2,0)
    T23=transformation(q[2,0],-np.pi/2,v3)
    T34=transformation(q[3,0],np.pi/2,0)
    T45=transformation(q[4,0],-np.pi/2,v5)
    T56=transformation(q[5,0],np.pi/2,0)
    T67=transformation(q[6,0],0,v7)
    #print("T01 =\n", T01)
    #print("T12 =\n", T12)
    T02=T01@T12
    #print("T02 =\n", T02)
    T03=T01@T12@T23
    T04=T01@T12@T23@T34
    T05=T01@T12@T23@T34@T45
    T06=T01@T12@T23@T34@T45@T56
    T07=T01@T12@T23@T34@T45@T56@T67

    #vrací list matic, T07 v sobě má polohu koncového efektoru
    T00=np.eye(4)
    T_mat_list=[T00,T01,T02,T03,T04,T05,T06,T07]
    p_E=T_mat_list[7][:3,3]
    return p_E,T07


q=np.radians(np.array([[10],[10],[10],[10],[10],[10],[10]]))
print(forward_kinematics(q))
print(forward_kinematics_new(q))