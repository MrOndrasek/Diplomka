import math
import numpy as np

theta2=0.1






R12=np.array([[math.cos(theta2),0,-math.sin(theta2)],
                    [math.sin(theta2),0,math.cos(theta2)],
                    [0,-1,0]])


v2=1
h1=1
T12=np.array([[R12[0, 0],R12[0, 1],R12[0, 2],0]
                ,[R12[1, 0],R12[1, 1],R12[1, 2],v2]
                ,[R12[2, 0],R12[2, 1],R12[2, 2],h1]
                ,[0,0,0,1]])

print(T12)