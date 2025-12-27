from simpy import *
import random
import matplotlib.pyplot as plt
import numpy as np

class Hybrid_SYSTEM:
    def __init__(self, env):
        # TEMPS
        self.t = np.linspace(0, 200, 10000)  
        # ENVIRONEMENT
        self.env = env

        # CONDITION INITIAL
        self.mode = '0' 

        self.x_ = 0
        self.z_ = 0

        self.x_data = []
        self.z_data = []

        self.compteur = 0

        # EVENEMENT
        self.e1 = Event(env)
        self.e2 = Event(env)

    def dynamique_continu(self):
        while True:
            match self.mode:
                case '0':
                    self.x_ = self.t
                    self.z_ =50* np.cos(self.t)  

                    if self.compteur < len(self.z_):
                        self.z_data.append((self.t[self.compteur], self.z_[self.compteur]))
                    else:
                        self.z_data.append((self.t[self.compteur], self.z_[-1]))
                    
                    if self.compteur < len(self.x_):
                        self.x_data.append((self.t[self.compteur], self.x_[self.compteur]))
                    else:
                        self.x_data.append((self.t[self.compteur], self.x_[-1]))

                    if(self.x_[self.compteur])>10:
                        self.e1.succeed()
                    
                case '1':
                    self.x_ = self.t
                    self.z_ =10*np.sin(self.t)

                    if self.compteur < len(self.z_):
                        self.z_data.append((self.t[self.compteur], self.z_[self.compteur]))
                    else:
                        self.z_data.append((self.t[self.compteur], self.z_[-1]))
                    
                    if self.compteur < len(self.x_):
                        self.x_data.append((self.t[self.compteur], self.x_[self.compteur]))
                    else:
                        self.x_data.append((self.t[self.compteur], self.x_[-1]))

                
                            
            
            
        
          
            self.compteur += 1
            yield self.env.timeout(1) 

    def automate(self):
        while True:
            match self.mode:
                case '0':
                    print(f'Time: [{self.env.now}] --> mode[0]')
                    yield self.e1
                    self.mode = '1'
                case '1':
                    print(f'Time: [{self.env.now}] --> mode[1]')
                    yield self.e2
                  

                

def main():
    env = Environment()
    machine = Hybrid_SYSTEM(env)
    env.process(machine.dynamique_continu())
    env.process(machine.automate())

    env.run(until=1000)  

    temps_point = [point[0] for point in machine.x_data]
    x_values = [point[1] for point in machine.x_data]
    z_values = [point[1] for point in machine.z_data]

    fig = plt.figure(figsize=(15, 10))
    plt.subplot(1, 1, 1)
    plt.plot(x_values, z_values, marker='')
    plt.xlabel('x')
    plt.ylabel('z')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
