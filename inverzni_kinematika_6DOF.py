import numpy as np
import math
import random


def forward_kinematics(q): #q je list o delce 7 s natocenim kloubu ve stupnich, vyhodi polohu koncoveho efektoru a seznam matic pro IK
    theta1=q[0,0]
    theta2=-q[1,0]
    theta3=-q[2,0]
    theta4=q[3,0]
    theta5=0
    theta6=-q[4,0]
    theta7=-q[5,0]
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
    v7=126
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
    return T_mat_list,p_E,R07
"""fw_result=forward_kinematics(np.zeros((6,1)))
print(fw_result[1])
for i, T in enumerate(fw_result[0]):
    print(i, T[:3,3])"""

def inverse_kinematics(p_cil,T_mat_list,p_E,R_cil,R_E): #p_cil je list se souradnicemi x,y,z
    e_p=(p_cil-p_E ).reshape(3,1)/100
    R_err=R_cil @ R_E.T
    e_r = rotation_error(R_cil,R_E)
    e=e = np.vstack((e_p, e_r))
    J_mat=np.zeros([6,6])

    znamenko = [1, -1, -1, 1, -1, -1]

    for j,i in enumerate([0,1,2,3,5,6]):
        z = T_mat_list[i][:3,2] * znamenko[j]
        p = T_mat_list[i][:3,3]

        J_mat[:3,j] = np.cross(z, p_E-p)
        J_mat[3:,j] = z

        """print(i+1)
        print("z =", T_mat_list[i][:3,2])
        print("p =", T_mat_list[i][:3,3])
        print("Jv =", np.cross(T_mat_list[i][:3,2],
        p_E-T_mat_list[i][:3,3]))"""
    J_mat[:3,:] /= 100
    para=0.1
    q = J_mat.T @ np.linalg.solve(
        J_mat @ J_mat.T + para**2 * np.eye(6),
        e
    )
    """print(p_E)
    print(J_mat)"""
    return q

def rotation_error(R_cil, R_E):

    R_err = R_cil @ R_E.T

    cos_theta = (np.trace(R_err) - 1) / 2
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    theta = math.acos(cos_theta)

    # téměř žádné natočení
    if theta < 1e-8:
        e_r = 0.5 * np.array([
            R_err[2,1] - R_err[1,2],
            R_err[0,2] - R_err[2,0],
            R_err[1,0] - R_err[0,1]
        ])

    # natočení téměř 180°
    elif math.pi - theta < 1e-6:

        A = (R_err + np.eye(3)) / 2

        # složky osy
        axis = np.sqrt(np.maximum(np.diag(A), 0))

        # vybereme největší složku a podle ní určíme znaménka
        k = np.argmax(axis)

        if axis[k] > 1e-8:
            axis = A[k, :] / axis[k]

        axis = axis / np.linalg.norm(axis)

        e_r = theta * axis

    # běžný případ
    else:
        axis = np.array([
            R_err[2,1] - R_err[1,2],
            R_err[0,2] - R_err[2,0],
            R_err[1,0] - R_err[0,1]
        ]) / (2 * math.sin(theta))

        e_r = theta * axis

    return e_r.reshape(3,1)


def joint_calculation(p_cil,R_cil_model,q_0,max_iter,tolerance):
    for i in range(0,max_iter):
            fw_res=forward_kinematics(q_0)
            e_k=p_cil-fw_res[1]
            e_RR=R_cil_model-fw_res[2]
            
            if (abs(e_k)<tolerance).all() and (abs(e_RR)<tolerance).all():
                return True,q_0
            elif i==max_iter-1:
                return False
            else:
                ik_res=inverse_kinematics(p_cil,fw_res[0],fw_res[1],R_cil_model,fw_res[2])

            parametr = 0.02

            q_1 = q_0 + ik_res * parametr

            # každý kloub se omezí samostatně
            q_min=np.radians([-170,-120,-170,-120,-120,-175]).reshape(6,1)
            q_max=np.radians([170,120,170,120,120,175]).reshape(6,1)
            q_1 = np.clip(q_1, q_min, q_max)

            q_0 = q_1

            """if i % 1000 == 0:
                print("iterace:", i)
                print("pozice:", fw_res[1])
                print("chyba pozice:", e_k)
                print("chyba orientace:", e_RR)
                print("q [deg]:", np.degrees(q_0.ravel()))
                print("dq:", ik_res.ravel())
                print()"""



def generation_of_movements(number_of_motions):
    q_0 = np.radians([
    [0],
    [0],
    [0],
    [0],
    [0],
    [0]
    ])
    
    max_iter=6000
    tolerance=0.01
    p_cil=np.array([
        500,300,600
        ])
    R_cil = np.array([
        [-1,0,0],
        [0,0,1],
        [0,1,0]
    ])
    R_fix = np.array([
        [0,-1,0],
        [1, 0,0],
        [0, 0,1]
    ])
    R_cil_model = R_cil @ R_fix.T 
    i=0

    while i<number_of_motions:
        p_cil=np.array([random.randint(300,600),random.randint(200,500),random.randint(400,700)])
        result=joint_calculation(p_cil,R_cil_model,q_0,max_iter,tolerance)
        
        if result==False:
            continue
        else:
            i+=1
            q_0=result[1]
            print(f"[{math.degrees(q_0[0,0])}],[{math.degrees(q_0[1,0])}],[{math.degrees(q_0[2,0])}],[{-math.degrees(q_0[3,0])}],[{math.degrees(q_0[4,0])}],[{math.degrees(q_0[5,0])}]")

generation_of_movements(10)