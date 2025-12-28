#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~#
#               Lokmane Rahmoune M2ISTR             #
#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~#
#   Modélisation et Simulation d'un système Hybride

from simpy import *
import random
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from scipy.differentiate import derivative
from scipy.integrate import solve_ivp


class Hybrid_SYSTEM:

    def __init__(self, env,params):

        # TEMPS
        self.t = np.linspace(0, 10, 1000)#|----->Temps [0-200] qui est divisé en 10 000 échantillons

        # ENVIRONEMENT

        self.env = env #|---->Environement de simulation simpy

        # CONDITION INITIAL

#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-

        self.mode = '0' 
      
        self.x = 0   #x     |
        self.z= 0    #z     |----->Position
        self.theta=0 #theta |
 
        self.Vx=0    #xdot      |
        self.Vz=0    #zdot      |---->Vitesse
        self.Vtheta=0#thetadot  |

        self.Ax=0    #xdotdot     |
        self.Az=0    #zdotdot     |----->Accélération
        self.Atheta=0#thetadotdot |

        self.state=[self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta ]            
   

#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
  
        #tuple [(temps , veleur)......(temps_n,valeur_n)]

        self.x_data = []
        self.theta_data = []
        self.z_data = []
        self.Vx_data = []
        self.Vtheta_data = []
        self.Vz_data = []
        self.Ax_data = []
        self.Aytheta_data = []
        self.Az_data = []

#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-

        #Compteur
        self.compteur = 0

#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-

        # EVENEMENT
        self.e1 = Event(env)
        self.e2 = Event(env)
        self.e3 = Event(env)

#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-

        #COSTANTE_SYS
        self.params = params

        self.F1=0
        self.F2=0

        self.flag=0

#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
    
#####################################################
  
    def u(self):
        match self.mode:
            case '0':
              F1=15
              F2=15.0
            case '1':
              F1=15
              F2=15.001
            case'2':
              F1=15
              F2=15.01
        return [F1,F2]
    
#####################################################

    def drone_dynamics(self, t, state, params):
        
         self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = state
         
         self.F1, self.F2 = self.u()
         
         self.m, self.I, self.L, self.g = params["m"], params["I"], params["L"], params["g"]

         
         self.Ax = (-1 * (self.F1 + self.F2) * np.sin(self.theta)) / self.m
         self.Az = ((self.F1 + self.F2) * np.cos(self.theta)) / self.m - self.g
         self.Atheta = ((self.F2 - self.F1) * self.L) / (2 * self.I)

        
         dot_state = [self.Vx, self.Vz, self.Vtheta, self.Ax, self.Az, self.Atheta]

         return dot_state
########################################################################################################

   
   



########################################################################################################

    def dynamique_continu(self):

        while True:

            match self.mode:

                case '0':
                   
                    if(self.flag==0):
                        sol = solve_ivp(self.drone_dynamics, (0, 200), self.state,'RK45', self.t, args=(self.params,))
                        self.x, self.z, self.theta, self.Vx, self.Vy, self.Vtheta= sol.y
                        self.flag=1

                    print("t,z",self.t[self.compteur], self.z[self.compteur])
                    print("t,x",self.t[self.compteur], self.x[self.compteur])

                    if self.compteur < len(self.z):
                        self.z_data.append((self.z[self.compteur]))
                    else:
                        self.z_data.append(( self.z[-1]))#indice -1 dernier element du tab self.(...)

                    if self.compteur < len(self.x):
                        self.x_data.append((self.x[self.compteur]))
                    else:
                        self.x_data.append(( self.x[-1]))#indice -1 dernier element du tab self.(...)

                    if self.compteur < len(self.theta):
                        self.theta_data.append((self.theta[self.compteur]))
                    else:
                        self.theta_data.append((self.theta[-1]))#indice -1 dernier element du tab self.(...)

                    if(self.z[self.compteur]>60):
                        self.e1.succeed()
                    


                    
                   
                    
                  

                    #------------------------------------------------------------------------------
                    
                case '1':
                   
                    
                    if(self.flag==0):
                        sol = solve_ivp(self.drone_dynamics, (0, 200), self.state,'RK45', self.t, args=(self.params,))
                        self.x, self.z, self.theta, self.Vx, self.Vy, self.Vtheta= sol.y
                        self.flag=1

                    print("t,z",self.t[self.compteur], self.z[self.compteur])
                    print("t,x",self.t[self.compteur], self.x[self.compteur])

                    if self.compteur < len(self.z):
                        self.z_data.append((self.z[self.compteur]))
                    else:
                        self.z_data.append(( self.z[-1]))#indice -1 dernier element du tab self.(...)

                    if self.compteur < len(self.x):
                        self.x_data.append((self.x[self.compteur]))
                    else:
                        self.x_data.append(( self.x[-1]))#indice -1 dernier element du tab self.(...)

                    if self.compteur < len(self.theta):
                        self.theta_data.append((self.theta[self.compteur]))
                    else:
                        self.theta_data.append((self.theta[-1]))#indice -1 dernier element du tab self.(...)
                    
                    if(self.z[self.compteur]>160):
                        self.e2.succeed()
                    

                case '2':
                   
                    
                    if(self.flag==0):
                        sol = solve_ivp(self.drone_dynamics, (0, 200), self.state,'RK45', self.t, args=(self.params,))
                        self.x, self.z, self.theta, self.Vx, self.Vy, self.Vtheta= sol.y
                        self.flag=1

                    print("t,z",self.t[self.compteur], self.z[self.compteur])
                    print("t,x",self.t[self.compteur], self.x[self.compteur])

                    if self.compteur < len(self.z):
                        self.z_data.append((self.z[self.compteur]))
                    else:
                        self.z_data.append(( self.z[-1]))#indice -1 dernier element du tab self.(...)

                    if self.compteur < len(self.x):
                        self.x_data.append((self.x[self.compteur]))
                    else:
                        self.x_data.append(( self.x[-1]))#indice -1 dernier element du tab self.(...)

                    if self.compteur < len(self.theta):
                        self.theta_data.append((self.theta[self.compteur]))
                    else:
                        self.theta_data.append((self.theta[-1]))#indice -1 dernier element du tab self.(...)

                
                   
                            
            self.compteur += 1
            yield self.env.timeout(1) 

#######################################################################

    
#######################################################################
    def automate(self):
        while True:
            match self.mode:
                case '0':
                    print(f'Time: [{self.t[self.compteur]}] --> mode[0]')
                    yield self.e1
                    self.mode = '1'
                    
                case '1':
                    self.flag=0
                    print(f'Time: [{self.t[self.compteur]}] --> mode[1]')
                    yield self.e2
                    self.mode = '2'
                case '2':
                    self.flag=0
                    print(f'Time: [{self.t[self.compteur]}] --> mode[2]')
                    yield self.e3
                
            yield self.env.timeout(1) 
                  

                
#######################################################################
def plot_results(t, x, y, theta):
        fig, axs = plt.subplots(2, 2, figsize=(10, 10))
        plt.subplots_adjust(hspace=0.5)

        axs[0, 0].plot(t, x)
        axs[0, 0].set_title("Horizontal Position")
        axs[0, 0].set_xlabel("t (s)")
        axs[0, 0].set_ylabel("x (m)")

        axs[0, 1].plot(t, y)
        axs[0, 1].set_title("Vertical Position")
        axs[0, 1].set_xlabel("t (s)")
        axs[0, 1].set_ylabel("y (m)")

        axs[1, 0].plot(t, theta)
        axs[1, 0].set_title("Pitch Angle")
        axs[1, 0].set_xlabel("t (s)")
        axs[1, 0].set_ylabel("theta (rad)")

        axs[1, 1].plot(x, y)
        axs[1, 1].set_title("Trajectory")
        axs[1, 1].set_xlabel("x (m)")
        axs[1, 1].set_ylabel("y (m)")

        plt.savefig("results.png")
        plt.show()

def main():


    m = 2 #(kg)
    L = 2 #(m)
    g = 9.81 #(m/s^2)
    I = (1/12) *m * L**2 # moment of inertia (kg*m^2)

    params= params = {"m": m, "L": L, "g": g, "I": I}

    env = Environment()
    machine = Hybrid_SYSTEM(env,params)

    env.process(machine.dynamique_continu())
    env.process(machine.automate())
    
    # Correction de l'indentation
    env.run(until=1000)
    
    plot_results(machine.t,machine.x_data,machine.z_data,machine.theta_data)




if __name__ == "__main__":
    main()
