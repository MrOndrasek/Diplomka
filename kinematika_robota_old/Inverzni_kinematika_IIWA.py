import numpy as np
import math

def forward_kinematics(q): #q je list o delce 7 s natocenim kloubu ve stupnich, vyhodi polohu koncoveho efektoru a seznam matic pro IK
    theta1=q[0,0]
    theta2=q[1,0]
    theta3=q[2,0]
    theta4=q[3,0]
    theta5=q[4,0]
    theta6=q[5,0]
    theta7=q[6,0]

    #rotační matice
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

    #rozmery mechanismu(pozdeji upravit)
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

    #transformační matice
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

    #vrací list matic, T07 v sobě má polohu koncového efektoru
    T00=np.eye(4)
    T_mat_list=[T00,T01,T02,T03,T04,T05,T06,T07]
    p_E=T_mat_list[7][:3,3]
    return T_mat_list,p_E,R07

def inverse_kinematics(p_cil,T_mat_list,p_E,R_cil,R_E): #p_cil je list se souradnicemi x,y,z
    e_p=p_cil-p_E
    R_err=R_cil @ R_E.T
    e_r=1/2*np.array([[R_err[2,1]-R_err[1,2]],[R_err[0,2]-R_err[2,0]],[R_err[1,0]-R_err[0,1]]])
    e=[[e_p[0]],[e_p[1]],[e_p[2]],e_r[0],e_r[1],e_r[2]]
    J_mat=np.zeros([6,7])
    for i in range(0,len(T_mat_list)-1):
        skalar=np.cross(T_mat_list[i][:3,2],(p_E-T_mat_list[i][:3,3]))
        vektor=T_mat_list[i][:3,2]
        J=np.array([skalar[0],skalar[1],skalar[2],vektor[0],vektor[1],vektor[2]])
        J_mat[:,i]=J

    q=np.linalg.pinv(J_mat) @ e
    return q

"""e=0.01
p_cil=[1,1,1]
R_cil=np.array([[1,0,0],[0,-1,0],[0,0,1]])
q_0=np.array([[0],[0],[0],[0],[0],[0],[0]])
q_min=np.radians([-170,-120,-170,-120,-170,-120,-175]).reshape(7,1)
q_max=np.radians([170,120,170,120,170,120,175]).reshape(7,1)

while 1==1:
    fw_result=forward_kinematics(q_0)
    ik_result=inverse_kinematics(p_cil,fw_result[0],fw_result[1],R_cil,fw_result[2])
    parametr=0.5
    q_1=q_0+(ik_result)*parametr
    while np.any(q_1 < q_min) or np.any(q_1 > q_max):
        parametr=parametr/2
        q_1=q_0+(ik_result)*parametr
        print(parametr)
        print(q_1)


    fw_result=forward_kinematics(q_1)
    if np.abs(p_cil[0]-fw_result[1][0])<e and np.abs(p_cil[1]-fw_result[1][1])<e and np.abs(p_cil[2]-fw_result[1][2])<e:
        print(fw_result[1])
        for i in range(7):
            vysledek=(q_1[i,0])
            print(vysledek)
        break
    else:
        q_0=q_1"""
q = np.zeros((7,1))
_, _, R_E = forward_kinematics(q)

for i in range(7):
    dq = 0.0001
    q_test = q.copy()
    q_test[i,0] += dq

    _, _, R_test = forward_kinematics(q_test)

    R_dot = (R_test - R_E) / dq

    print("\nkloub", i+1)
    print(R_dot)