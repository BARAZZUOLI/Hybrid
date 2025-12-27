from simpy import *
import random
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.differentiate import derivative

class Hybrid_SYSTEM:
    def __init__(self, env):

        #ENVIRONEMENT
        self.env = env

        #CONDITION INITIAL
        self.mode = '0' 

        self.x_=0
        self.z_=50
        self.Vx=50
        self.Vz=0
        self.Ax=0
        self.Az=0
       
        self.m=10
        self.Rx=5
        self.U=10
        self.Rz=5
        self.z_before=0
        self.x_before=0

        self.x_data=[]
        self.z_data=[]
        self.Vx_data=[]
        self.Vz_data=[]
        self.Ax_data=[]
        self.Az_data=[]  

        self.flag=0
    
        #EVENEMENT
        self.e1 = Event(env)
        self.e2 = Event(env)   
        self.e3 = Event(env)
        self.e4 = Event(env) 
        self.e5 = Event(env) 
     
    def dynamique_continu(self):

        while True:

            match self.mode:
                case'0':
                    self.x_+=1
                    self.z_=self.z_
                    self.Vx=1
                    self.Vz=0
                    if self.x_>=20:
                        self.e1.succeed() 
                case'1':
                    self.x_+=1
                    self.z_ -=1
                    self.Vx=5
                    self.Vz=-3
                    if self.z_==5:
                        self.e2.succeed()
                        
                case'2':
                    self.x_+=1
                    self.z_+=np.cos(self.z_*10)
                    self.Vx=5
                    self.Vz=0

            self.flag+=1
            self.x_data.append((self.env.now, self.x_))
            self.z_data.append((self.env.now, self.z_))
            self.x_before=(self.z_data[self.flag-1][1])
            self.z_before=(self.z_data[self.flag-1][1])
           
          

            yield self.env.timeout(1) 

    def automate(self):

        while True:
            match self.mode:
                case '0':
                    print(f'Time: [{self.env.now}] --> mode[0] [x: {self.x_:.2f} z: {self.z_:.2f}]')
                    yield self.e1
                    self.mode = '1'

                case '1':
                    print(f'Time: [{self.env.now}] --> mode[1] [x: {self.x_:.2f} z: {self.z_:.2f}]')
                    yield self.e2
                    self.mode = '2'

                case '2':
                    print(f'Time: [{self.env.now}] --> mode[2] [x: {self.x_:.2f} z: {self.z_:.2f}]')
                    yield self.e3
                    self.mode = '3'

                case '3':
                    print(f'Time: [{self.env.now}] --> state[3] [x: {self.x_:.2f} z: {self.z_:.2f}]')
                    yield self.env.timeout(1)
            
            

def main():

    env = Environment()
    machine = Hybrid_SYSTEM(env)
    env.process(machine.dynamique_continu())
    env.process(machine.automate())


    env.run(until=100)  

    temps_point = [point[1] for point in machine.x_data]
    x_values = [point[1] for point in machine.x_data]
    z_values = [point[1] for point in machine.z_data]


       
    fig = plt.figure(figsize=(15, 10))
    plt.subplot(1, 1, 1)
    plt.plot(temps_point,z_values ,marker='o')
    plt.xlabel('x values')
    plt.ylabel('z values')
    plt.grid(True)
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    main()