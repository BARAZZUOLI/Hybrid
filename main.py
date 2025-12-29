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
import matplotlib.animation as animation

class Hybrid_SYSTEM:

    def __init__(self, env,params):

        # TEMPS
        self.t = np.linspace(0, 10, 1000)#| ---> Temps [0-10] qui est divisé en 1 000 échantillons

        # ENVIRONEMENT

        self.env = env #| ---> Environement de simulation simpy

        # CONDITION INITIAL

#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-

        self.mode = '0' 
      
        self.x = 0   #x     |
        self.z= 0    #z     | ---> Position
        self.theta=0 #theta |
 
        self.Vx=0    #xdot      |
        self.Vz=0    #zdot      | ---> Vitesse
        self.Vtheta=0#thetadot  |

        self.Ax=0    #xdotdot     |
        self.Az=0    #zdotdot     | ---> Accélération
        self.Atheta=0#thetadotdot |

        self.state=[self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta ] #| ---> Vecteur d'état          
   

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
        self.e1 = Event(env)#|
        self.e2 = Event(env)#| ---> EVENEMENT DE GARDE
        self.e3 = Event(env)#|
        self.e4 = Event(env)#|
        self.e5 = Event(env)#|

#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-

        #COSTANTE_SYS
        self.params = params

        self.F1=0#
        self.F2=0#

#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-

        self.flag=0

#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
   
#######################################################################
    def x_z_theta_append(self):
    
#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-   
        if self.compteur < len(self.z):
            self.z_data.append((self.z[self.compteur]))
        else:
            self.z_data.append(( self.z[-1]))#indice -1 dernier element du tab self.(...)
#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
        if self.compteur < len(self.x):
            self.x_data.append((self.x[self.compteur]))
        else:
            self.x_data.append(( self.x[-1]))#indice -1 dernier element du tab self.(...)
#~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
        if self.compteur < len(self.theta):
            self.theta_data.append((self.theta[self.compteur]))
        else:
            self.theta_data.append((self.theta[-1]))#indice -1 dernier element du tab self.(...)
#######################################################################
    def u(self):
        match self.mode:
            case '0':
              F1=50.01
              F2=50
            case '1':
              F1=50
              F2=50.01
           
        return [F1,F2]
#######################################################################
    def drone_dynamics(self, t, state, params):
        
         self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = state
         
         self.F1, self.F2 = self.u()
         
         self.m, self.I, self.L, self.g = params["m"], params["I"], params["L"], params["g"]

         
         self.Ax = (-1 * (self.F1 + self.F2) * np.sin(self.theta)) / self.m
         self.Az = ((self.F1 + self.F2) * np.cos(self.theta)) / self.m - self.g
         self.Atheta = ((self.F2 - self.F1) * self.L) / (2 * self.I)

        
         dot_state = [self.Vx, self.Vz, self.Vtheta, self.Ax, self.Az, self.Atheta]

         return dot_state
#######################################################################
    def event_function(self, t, y, params):
        return y[0] - 10

    event_function.terminal = True
    event_function.direction = 1

    def dynamique_continu(self):
          while True:
              match self.mode:
                  case '0':
                      if self.flag == 0:
                          
                          sol1 = solve_ivp(self.drone_dynamics,(0, 1000),self.state,method='RK45',t_eval=self.t,args=(self.params,),events=self.event_function,dense_output=True)

                          self.x,self.z,self.theta,self.Vx,self.Vy,self.Vtheta = sol1.y
                          self.x_data,self.z_data,self.theta_data=self.x,self.z,self.theta
                          if sol1.t_events[0].size > 0:
                              self.t_event = sol1.t_events[0][0]
                              self.y_event = sol1.y_events[0][0]
                              print(f"Événement détecté à t = {self.t_event:.2f}")
                              print(f"État au moment de l'événement: x = {self.y_event[0]:.2f}")
                              
                          else:
                              print("Événement non détecté dans la phase 1")
                
                         
                          self.flag = 1


                  
                  
              self.compteur += 1
              yield self.env.timeout(1)


#######################################################################
    def automate(self):
     while True:
         match self.mode:
             case '0':
                 print(' --> mode[0]')
                

         yield self.env.timeout(1)
 
#######################################################################
#######################################################################
#######################################################################

#######################################################################
def main():
    m = 2                #| ---> (kg)
    L = 2                #| ---> (m)
    g = 9.81             #| ---> (m/s^2)
    I = (1/12) *m * L**2 #| ---> moment of inertia (kg*m^2)

    params= params = {"m": m, "L": L, "g": g, "I": I}

    env = Environment()
    machine = Hybrid_SYSTEM(env,params)

    env.process(machine.dynamique_continu())
    env.process(machine.automate())
    
    # Correction de l'indentation
    env.run(until=1000)
    
    plt.plot(machine.x,machine.z)
    plt.xlabel('X')
    plt.ylabel('Z')
    plt.title('Z en fonction de X')
    plt.grid(True)
    plt.show()
    
#######################################################################
if __name__ == "__main__":
    main()
#######################################################################