from simpy import *
import random
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

class MachineState:
    def __init__(self, env):
        #ENVIRONEMENT
        self.env = env
        #CONDITION INITIAL
        self.state = '0' 
        self.x = 0 
        self.z = 50  
        self.v = 0
        self.theta = 0 
        self.m=10
        self.Rx=5
        self.U=10
        self.x_data = []  
        self.z_data = [] 
        self.v_data = []  
        self.theta_data = []  
        #EVENEMENT
        self.e1 = Event(env)
        self.e2 = Event(env)  
        self.e3 = Event(env) 
        self.e4 = Event(env) 
        self.e5 = Event(env) 
     
    def dynamique_continu(self):
        while True:

            match self.state:
                case'0':
                    self.x+=self.v
                    self.z =self.z
                    self.v +=(1/self.m)*(self.U-self.Rx)
                    self.theta=0
                    if self.x>=20 :
                        self.e1.succeed()

                case'1':
                    self.theta=-1
                    self.x+=self.v*np.cos(self.theta)
                    self.z +=self.v*np.sin(self.theta)
                    self.v +=(1/self.m)*((self.U-self.Rx)-(self.m*9.81*np.sin(self.theta)))
                    if self.x>=50 :
                        self.e2.succeed()
                 
                case'2':
                    self.theta=0
                    self.x+=self.v*np.cos(self.theta)
                    self.z +=self.v*np.sin(self.theta)
                    self.v +=0
                    
                case'3':
                    a=0
                case _:
                    a=0

            self.x_data.append((self.env.now, self.x))
            self.z_data.append((self.env.now, self.z))
            self.v_data.append((self.env.now, self.x))
            self.theta_data.append((self.env.now, self.z))
            yield self.env.timeout(1) 

    def automate(self):

        while True:
            match self.state:
                case '0':
                    print(f'Time: [{self.env.now}] --> state[0] [x: {self.x:.2f} z: {self.z:.2f}]')
                    yield self.e1
                    self.state = '1'

                case '1':
                    print(f'Time: [{self.env.now}] --> state[1] [x: {self.x:.2f} z: {self.z:.2f}]')
                    yield self.e2
                    self.state = '2'

                case '2':
                    print(f'Time: [{self.env.now}] --> state[2] [x: {self.x:.2f} z: {self.z:.2f}]')
                    yield self.e3
                    self.state = '3'

                case '3':
                    print(f'Time: [{self.env.now}] --> state[3] [x: {self.x:.2f} z: {self.z:.2f}]')
                    yield self.env.timeout(1)
            
            

def main():

    env = Environment()
    machine = MachineState(env)

    env.process(machine.automate())
    env.process(machine.dynamique_continu())

    env.run(until=100)  

    time_points = [point[0] for point in machine.x_data]
    x_values = [point[1] for point in machine.x_data]
    z_values = [point[1] for point in machine.z_data]


       
    fig = plt.figure(figsize=(15, 10))
    plt.subplot(1, 1, 1)
    plt.plot(x_values, z_values,marker='o')
    plt.xlabel('x values')
    plt.ylabel('z values')
    plt.grid(True)
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    main()