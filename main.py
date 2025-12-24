from simpy import *
import random
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

class MachineState:
    def __init__(self, env):

        #ENVIRONEMENT
        self.env = env
        #Condition initial
        self.state = '0' 
        self.x = 0 
        self.y=0 
        self.z = 20  
        #LIST_DATA
        self.x_data = [] 
        self.y_data = []  
        self.z_data = []  
        #EVENEMENT
        self.e1 = Event(env)
        self.e2 = Event(env)  
        self.e3 = Event(env) 
        self.e4 = Event(env) 
        self.e5 = Event(env) 
      
    def dynamique_continu(self):
        while True:

            if self.state == '0':
                self.x += 0.1
                self.z = self.z 
                self.y = 0
                if self.x >= 10:
                    self.e1.succeed()  
                    
            if self.state == '1':
                self.x += 0.1
                self.z -=(1/self.x)
                self.y += 0
                if self.z <= 10:
                  self.e2.succeed()  

            if self.state == '2':
                self.x +=0.1
                self.z -=(1/self.x)
                self.y =0
                if self.z <=0:
                     self.e3.succeed()  

            if self.state == '3':
                self.x +=0.1
                self.z +=1/self.x
                self.y = 0
                    
                    
              
            self.x_data.append((self.env.now, self.x))
            self.y_data.append((self.env.now, self.y))
            self.z_data.append((self.env.now, self.z))
            yield self.env.timeout(0.1) 

    def automate(self):
        while True:
            if self.state == '0':
                print(f'Time: [{self.env.now}] --> state[0] [x: {self.x:.2f} z: {self.z:.2f}]')
                yield self.e1 
                self.state='1'

            elif self.state == '1':
                print(f'Time: [{self.env.now}] --> state[1] [x: {self.x:.2f} z: {self.z:.2f}]')
                yield self.e2
                self.state='2'

            elif self.state == '2':
                print(f'Time: [{self.env.now}] --> state[2] [x: {self.x:.2f} z: {self.z:.2f}]')
                yield self.e3
                self.state='3'

            
            elif self.state == '3':
                print(f'Time: [{self.env.now}] --> state[3] [x: {self.x:.2f} z: {self.z:.2f}]')
                yield self.env.timeout(0.1)



def main():

    env = Environment()
    machine = MachineState(env)

    env.process(machine.automate())
    env.process(machine.dynamique_continu())

    env.run(until=100)  

    time_points = [point[0] for point in machine.x_data]
    x_values = [point[1] for point in machine.x_data]
    y_values = [point[1] for point in machine.y_data]
    z_values = [point[1] for point in machine.z_data]


       
    fig = plt.figure(figsize=(15, 10))


    plt.subplot(2, 2, 1)
    plt.plot(x_values, z_values)
    plt.xlabel('x values')
    plt.ylabel('z values')
    plt.grid(True)



# Afficher tous les graphiques
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    main()