
         #~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~#
         #                                                   #
         #               Lokmane Rahmoune M2ISTR             #     
         #                                                   #                        
         #~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~#
         #  Modélisation et Simulation d'un système Hybride  #
         #####################################################

"""
Le code suivant vise à modéliser un drone en 2D dans le plan Z et X. 
Le but est de simuler un système hybride, c'est-à-dire un système qui 
contient à la fois une partie continue et une partie discrète.

Ce code est encore dans sa phase de conception ; 
il n'est pas encore terminé et n'est pas complètement propre. 
En le consultant, veuillez prendre note de ces éléments.

Sources:

https://arxiv.org/pdf/2106.15134

https://github.com/fpelogia/drone-simulation.git

"""

#           Z
#           ^              
#           |              
#           |                   F1                    F2
#           |                   ^                     ^
#           |                   |                     |
#           |                   |                     |
#           |                   |                     |
#           |
#           |                   X-------[drone]-------X
#           |              [Left Motor]          [Right Motor]      
#           |                               
#           |              
#           |
#           |
#           |
#           |
#           |
#           ------------------------------------------------------->  X
#
#
#
#
########################################################################

from simpy import *
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import  solve_ivp
import matplotlib.animation as animation


class Hybrid_SYSTEM:

    def __init__(self, env, params):

        # Temps
        self.t_start=0
        self.t_end=100
        self.resolution=10000
        self.t_mode_0 = np.linspace(self.t_start, self.t_end,self.resolution)  
        self.t_mode_1=0

        # Environnement

        self.env = env  # Environnement de simulation simpy

        # Conditions initiales

        self.mode = "0"

        self.x = 0
        self.z = 0
        self.theta = 0

        self.Vx = 0
        self.Vz = 0
        self.Vtheta = 0

        self.Ax = 0
        self.Az = 0
        self.Atheta = 0

        self.state = [self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta]

        self.condition_initial=0

        # Données (listes pour tracés)

        self.x_data = []
        self.theta_data = []
        self.z_data = []

        self.Vx_data = []
        self.Vtheta_data = []
        self.Vz_data = []

        self.temps=[]
     
        self.t_mode_0_data = []
        self.t_mode_1_data = []

        # Compteur

        self.compteur = 0

        # Événements (gardes)

        self.e1 = Event(env)
        self.e2 = Event(env)
        self.e3 = Event(env)
        self.e4 = Event(env)
        self.e5 = Event(env)

        # Constantes système

        self.params = params

        self.F1 = 0
        self.F2 = 0

        self.flag = 0
        self.flag2 = 0


    def event_function(self, t, y, params):
        match self.mode:
            case "0":
                return y[1] - 3000
               
    event_function.terminal = True
    event_function.direction = 1
                
    def u(self):
        match self.mode:
            case "0":
                F1 = 50
                F2 = 50.001
            case "1":
                F1 =50.001 
                F2 = 50

        return [F1, F2]


    """Dynamique du système source: https://github.com/fpelogia/drone-simulation.git """
    def drone_dynamics(self, t, state, params):

        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = state

        self.F1, self.F2 = self.u()

        self.m = params["m"]
        self.I = params["I"]
        self.L = params["L"]
        self.g = params["g"]

        self.Ax = -((self.F1 + self.F2) * np.sin(self.theta)) / self.m
        self.Az = ((self.F1 + self.F2) * np.cos(self.theta)) / self.m - self.g
        self.Atheta = ((self.F2 - self.F1) * self.L) / (2 * self.I)

        dot_state = [self.Vx, self.Vz, self.Vtheta, self.Ax, self.Az, self.Atheta]

        return dot_state
        

    def dynamique_continu(self):
        while True:

            match self.mode:

                case "0":
                    
                    
                    """Mode 0"""

                    if self.flag == 0:#|---> Pour éviter de faire intégration numerique plusieur fois 
                        self.flag = 1 #|

                        """Integration numérique """

                        sol1 = solve_ivp(
                            self.drone_dynamics,# Dynamique du système
                            (self.t_start,  self.t_end),# intervalle de temps intégration numérique
                            self.state,# états du système (xdotdot,zdotdot,thetadotdot)
                            method="RK45",# Méthode d'intégration
                            t_eval=self.t_mode_0,# tab-->temps 
                            args=(self.params,),#constante du système
                            events=self.event_function,#Événement pour arrêter l'intégration numérique 
                            dense_output=True,#S'il faut calculer une solution continue. La valeur par défaut est False 
                        )

                        """Mise à jour  des états aprés integration numérique """

                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = sol1.y  

                        """Stockage"""

                        self.x_data = self.x            
                        self.z_data = self.z            
                        self.theta_data = self.theta    
                        self.t_mode_0_data = sol1.t 
                    
                        """Vérification si événement a eu lieu """

                        if sol1.t_events[0].size > 0:

                            self.e1.succeed()
                            self.t_event = sol1.t_events[0][0]
                            self.y_event = sol1.y_events[0][0]
                            print(f"Événement détecté à t = {self.t_event:.2f}")
                            print(f"État au moment de l'événement: z = {self.y_event[1]:.2f}")

                            # État initial pour le mode 2

                            self.condition_initial = self.y_event
                            self.t_start = self.t_event

                        else:
                            print("Événement non détecté dans mode 1")

                        

                case "1":

                    """Mode 1"""

                    if self.flag == 1:#|---> Pour éviter de faire intégration numerique plusieur fois 
                        self.flag = 2 #|

                        self.t2=np.linspace(self.t_start, self.t_end, self.resolution)
                       
                        sol2 = solve_ivp(
                            self.drone_dynamics,
                            (self.t_start,  self.t_end),
                            self.condition_initial,
                            method="RK45",
                            t_eval=self.t2,
                            args=(self.params,),
                        )

                        """mise à jour  des états aprés integration numérique """
                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = sol2.y  
                    
                        """stockage"""
                        self.x_data = np.concatenate((self.x_data, self.x))  
                        self.z_data = np.concatenate((self.z_data, self.z))            
                        self.theta_data = np.concatenate((self.theta_data, self.theta))    
                        self.t_mode_1_data = sol2.t 
                        self.temps=np.concatenate((self.t_mode_0_data, self.t_mode_1_data  ))
                     
                     
            self.compteur += 1
            yield self.env.timeout(1)

    def automate(self):
        while True:
            match self.mode:
               
                case "0":

                    """Mode 0"""

                    if self.flag2 == 0:#|--> Pour éviter de print le mode 0 plusieur fois 
                        self.flag2 =1  #|

                    print("--> mode[0]<--")


                    """Attente de l'évenement e1"""

                    yield self.e1

                    self.flag = 1
                    self.mode = "1"

                case "1":

                    """Mode 1"""

                    if self.flag2==1:
                        self.flag2 =0

                        print("--> mode[1]<--")

            yield self.env.timeout(1)


def plot_results(t, x, y, theta):

    fig, axs = plt.subplots(2, 2, figsize=(10, 10))
    plt.subplots_adjust(hspace=0.5)

    axs[0, 0].plot(t, x)
    axs[0, 0].set_title("Position Horizontale")
    axs[0, 0].set_xlabel("t (s)")
    axs[0, 0].set_ylabel("x (m)")

    axs[0, 1].plot(t, y)
    axs[0, 1].set_title("Position Verticale")
    axs[0, 1].set_xlabel("t (s)")
    axs[0, 1].set_ylabel("z (m)")

    axs[1, 0].plot(t, theta)
    axs[1, 0].set_title("Angle θ")
    axs[1, 0].set_xlabel("t (s)")
    axs[1, 0].set_ylabel("θ (rad)")

    axs[1, 1].plot(x, y)
    axs[1, 1].set_title("Trajectoire")
    axs[1, 1].set_xlabel("x (m)")
    axs[1, 1].set_ylabel("z (m)")

    
    for ax in axs.flat:
        ax.grid()
    plt.savefig("results.png")
    plt.show()

def main():
    m = 2  # kg
    L = 2  # m
    g = 9.81  # m/s^2
    I = (1 / 12) * m * L ** 2  # moment of inertia (kg*m^2)

    params = {"m": m, "L": L, "g": g, "I": I}

    env = Environment()
    machine = Hybrid_SYSTEM(env, params)

    env.process(machine.dynamique_continu())
    env.process(machine.automate())

    env.run(until=machine.resolution)

   
    plot_results(machine.temps,machine.x_data,machine.z_data,machine.theta_data)
   # plot_trajectory(machine.x_data,machine.z_data,machine.theta_data,machine.t_end,params)

  



if __name__ == "__main__":
    main()
