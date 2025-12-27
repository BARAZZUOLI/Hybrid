from simpy import *
import random
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from scipy.differentiate import derivative


class Hybrid_SYSTEM:


############################################################
    def __init__(self, env):
        # TEMPS
        self.t = np.linspace(0, 200, 10000)  
        # ENVIRONEMENT
        self.env = env
        # CONDITION INITIAL
#---------------------------------------------------
        self.mode = '0' 
#---------------------------------------------------
        self.x_ = 0
        self.y=0
        self.z_ = 50
#---------------------------------------------------     
        self.Vx=0
        self.Vy=0
        self.Vz=0
#---------------------------------------------------
        self.Ax=0
        self.Ay=0
        self.Az=0
#---------------------------------------------------       
        #tuple [(temps , veleur)......(temps_n,valeur_n)]
        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.Vx_data = []
        self.Vy_data = []
        self.Vz_data = []
        self.Ax_data = []
        self.Ay_data = []
        self.Az_data = []
        #Compteur
        self.compteur = 0
        # EVENEMENT
        self.e1 = Event(env)
        self.e2 = Event(env)
        #COSTANTE_SYS
        self.k= 10
        self.c = 5
        self.m = 5   
######################################################################

    def dynamique_mode_0(self):
        #à compléter

    def dynamique_mode_1 (self):
        #à compléter

    def dynamique_mode_2 (self):
        #à compléter
#######################################################################

    def dynamique_continu(self):
        while True:
            match self.mode:
                case '0':
                    self.dynamique_mode_0()
                    #------------------------------------------------------------------------------
                    if self.compteur < len(self.z_):
                        self.z_data.append((self.t[self.compteur], self.z_[self.compteur]))
                    else:
                        self.z_data.append((self.t[self.compteur], self.z_[-1]))#indice -1 dernier element du tab self.(...)
                    #------------------------------------------------------------------------------
                    if self.compteur < len(self.x_):
                        self.x_data.append((self.t[self.compteur], self.x_[self.compteur]))
                    else:
                        self.x_data.append((self.t[self.compteur], self.x_[-1]))#indice -1 dernier element du tab self.(...)
                    #------------------------------------------------------------------------------
                    if(self.x_[self.compteur])>5:
                        self.e1.succeed()
                    
                case '1':
                    self.dynamique_mode_1()
                    #------------------------------------------------------------------------------
                    if self.compteur < len(self.z_):
                        self.z_data.append((self.t[self.compteur], self.z_[self.compteur]))
                    else:
                        self.z_data.append((self.t[self.compteur], self.z_[-1]))#indice -1 dernier element du tab self.(...)
                    #------------------------------------------------------------------------------
                    if self.compteur < len(self.x_):
                        self.x_data.append((self.t[self.compteur], self.x_[self.compteur]))
                    else:
                        self.x_data.append((self.t[self.compteur], self.x_[-1]))#indice -1 dernier element du tab self.(...)
                    #------------------------------------------------------------------------------
                    if(self.z_[self.compteur])<20:
                        self.e2.succeed()

                case '2':
                    self.dynamique_mode_2()
                    #------------------------------------------------------------------------------
                    if self.compteur < len(self.z_):
                        self.z_data.append((self.t[self.compteur], self.z_[self.compteur]))
                    else:
                        self.z_data.append((self.t[self.compteur], self.z_[-1]))#indice -1 dernier element du tab self.(...)
                    #------------------------------------------------------------------------------
                    if self.compteur < len(self.x_):
                        self.x_data.append((self.t[self.compteur], self.x_[self.compteur]))
                    else:
                        self.x_data.append((self.t[self.compteur], self.x_[-1]))#indice -1 dernier element du tab self.(...)
                   
                            
            self.compteur += 1
            yield self.env.timeout(1) 

#######################################################################

    def automate(self):
        while True:
            match self.mode:
                case '0':
                    print(f'Time: [{self.t[self.compteur]}] --> mode[0]')
                    yield self.e1
                    self.mode = '1'
                case '1':
                    print(f'Time: [{self.t[self.compteur]}] --> mode[1]')
                    yield self.e2
                    self.mode = '2'
                case '2':
                    print(f'Time: [{self.t[self.compteur]}] --> mode[2]')
                
            yield self.env.timeout(1) 
                  

                
#######################################################################


def main():
    env = Environment()
    machine = Hybrid_SYSTEM(env)
    env.process(machine.dynamique_continu())
    env.process(machine.automate())

    env.run(until=700)  

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
