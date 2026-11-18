import math
import numpy as np
import h5py

with h5py.File("diplomka_complete.h5","r") as f:
    measurement=f["measurements/measurement_4"]

    q_wanted=measurement["q_wanted"][:]

    q_indicated=measurement["q_indicated"][:]

    distance=measurement["distance"][:]
    angle_Hz=measurement["angle_hz"][:]
    angle_Vt=measurement["angle_vt"][:]
    mereni_leika=[distance,angle_Hz,angle_Vt]

    timestamp=measurement["timestamp"][:]


def leika_measurement_to_coordinates(i):
    d=distance[i]
    angle_hz=angle_Vt[i]
    angle_vt=angle_Hz[i]

    distance_in_mm=d*1000
    x=distance_in_mm*math.sin(angle_vt)*math.cos(angle_hz)
    y=distance_in_mm*math.sin(angle_vt)*math.sin(angle_hz)
    z=distance_in_mm*math.cos(angle_vt)

    p=np.array([x,y,z])
    return p

def robot_measurement_to_coordinates(i): #q je list o delce 7 s natocenim kloubu ve stupnich, vyhodi polohu koncoveho efektoru a seznam matic pro IK
    q=np.radians(q_wanted[i])
    theta1=q[0]
    theta2=-q[1]
    theta3=-q[2]
    theta4=-q[3]
    theta5=q[4]
    theta6=-q[5]
    theta7=-q[6]
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

    TCP_offset=51
    T_TCP = np.eye(4)
    T_TCP[:3, 3] = [0, 0, TCP_offset]

    T_0_TCP = T07 @ T_TCP
    p_TCP = T_0_TCP[:3, 3]
    #vrací list matic, T07 v sobě má polohu koncového efektoru
    T00=np.eye(4)
    T_mat_list=[T00,T01,T02,T03,T04,T05,T06,T07]
    p_E=T_mat_list[7][:3,3]
    return p_TCP

def robot_indicated_measurement_to_coordinates(i): #q je list o delce 7 s natocenim kloubu ve stupnich, vyhodi polohu koncoveho efektoru a seznam matic pro IK
    q=np.radians(q_indicated[i])
    theta1=q[0]
    theta2=-q[1]
    theta3=-q[2]
    theta4=-q[3]
    theta5=q[4]
    theta6=-q[5]
    theta7=-q[6]
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

    TCP_offset=51
    T_TCP = np.eye(4)
    T_TCP[:3, 3] = [0, 0, TCP_offset]

    T_0_TCP = T07 @ T_TCP
    p_TCP = T_0_TCP[:3, 3]
    #vrací list matic, T07 v sobě má polohu koncového efektoru
    T00=np.eye(4)
    T_mat_list=[T00,T01,T02,T03,T04,T05,T06,T07]
    p_E=T_mat_list[7][:3,3]
    return p_TCP

def measurement_points_lists(lo_index,hi_index):
    robot_points=[]
    robot_points_indicated=[]
    leika_points=[]
    for i in range(lo_index,hi_index):
        if leika_measurement_to_coordinates(i).all()==0:
            continue
        else:
            robot_points.append(robot_measurement_to_coordinates(i))
            leika_points.append(leika_measurement_to_coordinates(i))
            robot_points_indicated.append(robot_indicated_measurement_to_coordinates(i))
    return robot_points, leika_points, robot_points_indicated



def transformation_matrix(T1,T2,T3):
    rT2=T2-T1
    rT3=T3-T1
    iT=(rT2)/np.linalg.norm(rT2)
    kT=np.cross(rT2,rT3)/np.linalg.norm(np.cross(rT2,rT3))
    jT=np.cross(kT,iT)/np.linalg.norm(np.cross(kT,iT))
    R=np.column_stack((iT,kT,jT))
    matrix=np.eye(4)
    matrix[:3,:3]=R
    matrix[:3,3]=T1
    print(matrix)
    return matrix


def robot_base_method_one(T1_index,T2_index,T3_index):
    T1_robot=robot_measurement_to_coordinates(T1_index)
    T2_robot=robot_measurement_to_coordinates(T2_index)
    T3_robot=robot_measurement_to_coordinates(T3_index)

    T1_leika=leika_measurement_to_coordinates(T1_index)
    T2_leika=leika_measurement_to_coordinates(T2_index)
    T3_leika=leika_measurement_to_coordinates(T3_index)

    robot_transformation=transformation_matrix(T1_robot,T2_robot,T3_robot)
    leika_transformation=transformation_matrix(T1_leika,T2_leika,T3_leika)

    R=robot_transformation@np.linalg.inv(leika_transformation)
    return R


def least_square_method(robot_points_list,leika_points_list):
    robot_points=np.array(robot_points_list)
    leika_points=np.array(leika_points_list)
    robot_schwerpunkt=np.mean(robot_points_list,axis=0)

    leika_schwerpunkt=np.mean(leika_points_list,axis=0)

    robot_points_centered=robot_points_list-robot_schwerpunkt
    leika_points_centered=leika_points_list-leika_schwerpunkt
            
    H=robot_points_centered.T @ leika_points_centered
    U,S,Vt= np.linalg.svd(H)

    D = np.eye(3)
    D[2, 2] = np.linalg.det(Vt.T @ U.T)

    R = Vt.T @ D @ U.T

    t=leika_schwerpunkt-R @ robot_schwerpunkt

    transformed_point=(R @ robot_points.T).T + t
    errors=np.linalg.norm(leika_points-transformed_point,axis=1)
    RMSE=np.sqrt(np.mean(errors**2))
    print("matice R:",R,"tranformace:",t)
    print("RMSE:",RMSE,"mm")
    return R,t


def robot_dimensions(q_in,dimensions_delta): 
    q=np.radians(q_in)
    theta1=q[0]
    theta2=-q[1]
    theta3=-q[2]
    theta4=-q[3]
    theta5=q[4]
    theta6=-q[5]
    theta7=-q[6]
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

    #rozmery mechanismu(pozdeji upravit)
    
    v1=360+dimensions_delta[0]
    v2=0+dimensions_delta[1]
    v3=420+dimensions_delta[2]
    v4=0+dimensions_delta[3]
    v5=400+dimensions_delta[4]
    v6=0+dimensions_delta[5]
    v7=126+dimensions_delta[6]
    h1=0+dimensions_delta[7]
    h2=0+dimensions_delta[8]
    h3=0+dimensions_delta[9]

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
    return p_E

def objective(delta,R,t):
    q=q_wanted
    measured=R*measurement_points_lists(0,1800)[1]+t
    predicted=np.array([
        robot_dimensions(qi,delta) for qi in q
    ])

    return np.sqrt(np.mean(np.sum((predicted-measured)**2,axis=1)))
    
    
list_of_points=measurement_points_lists(0,1800)
transformation=least_square_method(list_of_points[0],list_of_points[1])
