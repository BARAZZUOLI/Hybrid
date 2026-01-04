# ~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~#
#                                                   #
#               Lokmane Rahmoune M2ISTR             #
#                                                   #
# ~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~#
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
from scipy.integrate import solve_ivp
import matplotlib.animation as animation
from math import pi 
import random


class Hybrid_SYSTEM:

    def __init__(self, env, params):

        # Temps

        self.t_start = 0
        self.t_end = 6
        self.resolution = 1000000
        self.t_mode_0 = 0
        self.t_mode_1 = 0
        self.t_mode_2 = 0
        self.t_mode_3 = 0
        self.t_mode_4 = 0
        self.t_mode_5 = 0
        self.t_mode_6 = 0
        self.t_mode_7 = 0

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

        self.condition_initial = [
            self.x,
            self.z,
            self.theta,
            self.Vx,
            self.Vz,
            self.Vtheta,
        ]

        # Données (listes pour tracés)

        self.x_data = []
        self.theta_data = []
        self.z_data = []

        self.Vx_data = []
        self.Vtheta_data = []
        self.Vz_data = []

        self.temps = []

        self.t_mode_0_data = []
        self.t_mode_1_data = []
        self.t_mode_2_data = []
        self.t_mode_3_data = []
        self.t_mode_4_data = []
        self.t_mode_5_data = []
        self.t_mode_6_data = []
        self.t_mode_7_data = []

        # Compteur

        self.compteur = 0

        # Événements (gardes)

        self.e0 = Event(env)
        self.e1 = Event(env)
        self.e2 = Event(env)
        self.e3 = Event(env)
        self.e4 = Event(env)
        self.e5 = Event(env)
        self.e6 = Event(env)
        self.e7 = Event(env)
        self.e8 = Event(env)
        self.e9 = Event(env)

        # Constantes système

        self.params = params

        self.F1 = 0
        self.F2 = 0
        self.F3 = 0
        self.F4 = 0
        self.FP = 0

        self.flag = 0
        self.flag2 = 0
        self.flag3=0

        self.F_g = (self.params["m"] * self.params["g"]/2)  # Force de gravité divisée par 2 pour chaque moteur
        


    def u(self, t):
        # Initialisation par défaut
        F1 = F2 = F3 = F4 = FP = 0
        
        match self.mode:
            case "0":
                F1 = self.F_g
                F2 = self.F_g
                F3 = 0
                F4 = 0
                FP=0
            case "1":
                F1 = 10*self.F_g + 0.01
                F2 = 10*self.F_g
                F3 = 0
                F4 = 0
                FP=0
            case "2":
                F1 = 10*self.F_g
                F2 = 10*self.F_g + 0.01
                F3 = 0
                F4 = 0
                FP=0
            case "3":
            
                F1=self.F_g
                F2=self.F_g
                F3=0.01 
                F4=0
                FP=0
            case "4":
                F1 = -self.F_g
                F2 = -self.F_g-0.01
                F3 = 0
                F4 = 0
                FP=0
            case "5":
                F1 = -self.F_g-0.01
                F2 = -self.F_g
                F3 = 0
                F4 = 0
                FP = 0
            case '6':
                F1 = self.F_g
                F2 = self.F_g
                F3 = 0
                F4 = 0
                FP = 0
            case '7':
                F1=self.F_g
                F2=self.F_g
                F3=0.01 
                F4=0
                FP=10



        return [F1, F2, F3, F4,FP]

    """Dynamique du système source: https://github.com/fpelogia/drone-simulation.git """

    def drone_dynamics(self, t, state, params):

        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = state

        self.F1, self.F2, self.F3, self.F4,self.FP = self.u(t)

        self.m = params["m"]
        self.I = params["I"]
        self.L = params["L"]
        self.g = params["g"]

        self.Ax = -(((self.F1 + self.F2) * np.sin(self.theta)) / self.m )+((self.F3 - self.F4)/self.m)
            
        self.Az = (((self.F1 + self.F2) * np.cos(self.theta)) / self.m) - (self.g) -(self.FP/self.m)

        self.Atheta = ((self.F2 - self.F1) * self.L) / (2 * self.I)

        dot_state = [self.Vx, self.Vz, self.Vtheta, self.Ax, self.Az, self.Atheta]

        return dot_state
    
    def theta_sup_pos_0_01_rad(self, t, y, params):
        return y[2] + 0.01

    theta_sup_pos_0_01_rad.terminal= True # type: ignore
    theta_sup_pos_0_01_rad.direction = 1 # type: ignore

    def theta_inf_neg_0_01_rad(self, t, y, params):
        return y[2] + 0.01

    theta_inf_neg_0_01_rad.terminal = True # type: ignore
    theta_inf_neg_0_01_rad.direction = -1 # type: ignore

    def x_sup_val(self, t, y, params):
        return y[0] - 35
    
    x_sup_val.terminal = True # type: ignore
    x_sup_val.direction = 1 # type: ignore

    def perturbation(self, t, y, params):
        return y[0] - 25

    perturbation.terminal = True # type: ignore
    perturbation.direction = 1 # type: ignore



    def z_egal_40(self, t, y, params):
        return y[1] - 40

    z_egal_40.terminal = True # type: ignore
    z_egal_40.direction = -1 # type: ignore

    
    """Dynamique continue du système hybride """

    def dynamique_continu(self):
        while True:
            match self.mode:
                case "0":
                    """Mode 0"""
                    self.t_start = self.compteur * (1 / self.resolution)
                    if self.compteur == (self.resolution - (self.resolution - 10)):
                        self.e0.succeed()

                case "1":
                    if self.flag == 0:
                        self.flag = 1
                        self.t_mode_1 = np.linspace(self.t_start, self.t_end, self.resolution)
                        sol1 = solve_ivp(
                            self.drone_dynamics,
                            (self.t_start, self.t_end),
                            self.condition_initial,
                            method="RK45",
                            t_eval=self.t_mode_1,
                            args=(self.params,),
                            events=self.theta_inf_neg_0_01_rad,
                            dense_output=True,
                        )
                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = sol1.y
                        
                       
                        self.x_data = np.concatenate((self.x_data, self.x))
                        self.z_data = np.concatenate((self.z_data, self.z))
                        self.theta_data = np.concatenate((self.theta_data, self.theta))
                        self.Vx_data = np.concatenate((self.Vx_data, self.Vx))
                        self.Vz_data = np.concatenate((self.Vz_data, self.Vz))
                        self.Vtheta_data = np.concatenate((self.Vtheta_data, self.Vtheta))
                        
                        self.t_mode_1_data = sol1.t
                        self.temps = np.concatenate((self.temps, self.t_mode_1_data))

                        if len(sol1.t_events) > 0 and sol1.t_events[0].size > 0:
                            self.t_event = sol1.t_events[0][0]
                            self.y_event = sol1.y_events[0][0]
                            if self.y_event[1] > 50:
                                self.e3.succeed()
                        
                                self.t_start = self.t_event
                            else:
                                self.e1.succeed()
                            print(f"mode 1 Événement détecté à t = {self.t_event:.2f}, x = {self.y_event[0]:.2f}, z = {self.y_event[1]:.2f}, theta = {self.y_event[2]:.2f}")
                            self.condition_initial = self.y_event
                            self.t_start = self.t_event
                        else:
                            print("Événement non détecté dans mode 1")

                case "2":
                    """Mode 2"""
                    if self.flag == 1:
                        self.flag = 0
                        self.t_mode_2 = np.linspace(self.t_start, self.t_end, self.resolution)
                        sol2 = solve_ivp(
                            self.drone_dynamics,
                            (self.t_start, self.t_end),
                            self.condition_initial,
                            method="RK45",
                            t_eval=self.t_mode_2,
                            args=(self.params,),
                            events=self.theta_sup_pos_0_01_rad,
                            dense_output=True,
                        )
                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = sol2.y
                    
                        self.x_data = np.concatenate((self.x_data, self.x))
                        self.z_data = np.concatenate((self.z_data, self.z))
                        self.theta_data = np.concatenate((self.theta_data, self.theta))
                        self.Vx_data = np.concatenate((self.Vx_data, self.Vx))
                        self.Vz_data = np.concatenate((self.Vz_data, self.Vz))
                        self.Vtheta_data = np.concatenate((self.Vtheta_data, self.Vtheta))
                        
                        self.t_mode_2_data = sol2.t
                        self.temps = np.concatenate((self.temps, self.t_mode_2_data))

                        if len(sol2.t_events) > 0 and sol2.t_events[0].size > 0:
                            self.t_event = sol2.t_events[0][0]
                            self.y_event = sol2.y_events[0][0]
                            if self.y_event[1] > 50:
                                self.e3.succeed()
                                
                                self.t_start = self.t_event
                            else:
                                self.e2.succeed()
                            print(f"mode 2 Événement détecté à t = {self.t_event:.2f}, x = {self.y_event[0]:.2f}, z = {self.y_event[1]:.2f}, theta = {self.y_event[2]:.2f}")
                           
                            self.condition_initial = self.y_event
                            self.t_start = self.t_event
                        else:
                            print("Événement non détecté dans mode 2")

                case "3":
                    """Mode 3"""
                    if self.flag == 0 or self.flag == 1:
                         self.flag = 5
                         self.condition_initial = self.y_event[0], self.y_event[1], 0, 0, 0, 0
                         self.t_mode_3 = np.linspace(self.t_start, self.t_end, self.resolution)

                         sol3 = solve_ivp(
                             self.drone_dynamics,
                             (self.t_start, self.t_end),
                             self.condition_initial,
                             method="RK45",
                             t_eval=self.t_mode_3,
                             args=(self.params,),
                             events=[self.x_sup_val, self.perturbation],
                             dense_output=True,
                         )

                      
                         self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = sol3.y

                         self.x_data = np.concatenate((self.x_data, self.x))
                         self.z_data = np.concatenate((self.z_data, self.z))
                         self.theta_data = np.concatenate((self.theta_data, self.theta))
                         self.Vx_data = np.concatenate((self.Vx_data, self.Vx))
                         self.Vz_data = np.concatenate((self.Vz_data, self.Vz))
                         self.Vtheta_data = np.concatenate((self.Vtheta_data, self.Vtheta))

                         self.t_mode_3_data = sol3.t
                         self.temps = np.concatenate((self.temps, self.t_mode_3_data))

                         # Gestion des événements détectés
                         event_detected = False
                         if len(sol3.t_events) > 0:
                             for idx, event in enumerate(sol3.t_events):
                                 if event.size > 0:
                                     self.t_event = event[0]
                                     self.y_event = sol3.y_events[idx][0]
                                     event_detected = True

                                     if idx == 1:  # Première condition d'événement
                                         self.e8.succeed()
                                         self.flag = 9
                                         self.condition_initial = self.y_event
                                         self.t_start = self.t_event
                                     elif idx == 0:  # Deuxième condition d'événement
                                         self.e4.succeed()
                                         print(f"mode 3 Événement perturbation détecté à t = {self.t_event:.2f}, x = {self.y_event[0]:.2f}, z = {self.y_event[1]:.2f}, theta = {self.y_event[2]:.2f}")
                                         self.condition_initial = self.y_event
                                         self.t_start = self.t_event
                                     break
                                 
                         if not event_detected:
                             print("Aucun événement détecté dans mode 3")


                case "4":
                    """Mode 4 """
                    if self.flag == 5:
                        self.flag = 6
                        self.t_mode_4 = np.linspace(self.t_start, self.t_end, self.resolution)
                        sol4 = solve_ivp(
                            self.drone_dynamics,
                            (self.t_start, self.t_end),
                            self.condition_initial,
                            method="RK45",
                            t_eval=self.t_mode_4,
                            args=(self.params,),
                            events=self.theta_inf_neg_0_01_rad,
                            dense_output=True,
                        )
                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = sol4.y

                        self.x_data = np.concatenate((self.x_data, self.x))
                        self.z_data = np.concatenate((self.z_data, self.z))
                        self.theta_data = np.concatenate((self.theta_data, self.theta))
                        self.Vx_data = np.concatenate((self.Vx_data, self.Vx))
                        self.Vz_data = np.concatenate((self.Vz_data, self.Vz))
                        self.Vtheta_data = np.concatenate((self.Vtheta_data, self.Vtheta))

                        self.t_mode_4_data = sol4.t
                        self.temps = np.concatenate((self.temps, self.t_mode_4_data))

                        if len(sol4.t_events) > 0 and sol4.t_events[0].size > 0:
                            self.t_event = sol4.t_events[0][0]
                            self.y_event = sol4.y_events[0][0]
                            if( self.y_event[1]<5):
                                self.e7.succeed()
                                self.flag = 7
                                
                                self.t_start = self.t_event
                            else:
                                self.e5.succeed()
                            print(f"mode 4 Événement détecté à t = {self.t_event:.2f}, x = {self.y_event[0]:.2f}, z = {self.y_event[1]:.2f}, theta = {self.y_event[2]:.2f}")
                            self.condition_initial = self.y_event
                            self.t_start = self.t_event
                        else:
                            print("Événement non détecté dans mode 4")

                case "5":
                    """Mode 5 - Descente 2"""
                    if self.flag == 6:
                        self.flag = 5
                        self.t_mode_5 = np.linspace(self.t_start, self.t_end, self.resolution)
                        sol5 = solve_ivp(
                            self.drone_dynamics,
                            (self.t_start, self.t_end),
                            self.condition_initial,
                            method="RK45",
                            t_eval=self.t_mode_5,
                            args=(self.params,),
                            events=self.theta_sup_pos_0_01_rad,
                            dense_output=True,
                        )
                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = sol5.y

                        self.x_data = np.concatenate((self.x_data, self.x))
                        self.z_data = np.concatenate((self.z_data, self.z))
                        self.theta_data = np.concatenate((self.theta_data, self.theta))
                        self.Vx_data = np.concatenate((self.Vx_data, self.Vx))
                        self.Vz_data = np.concatenate((self.Vz_data, self.Vz))
                        self.Vtheta_data = np.concatenate((self.Vtheta_data, self.Vtheta))

                        self.t_mode_5_data = sol5.t
                        self.temps = np.concatenate((self.temps, self.t_mode_5_data))

                        if len(sol5.t_events) > 0 and sol5.t_events[0].size > 0:
                            self.t_event = sol5.t_events[0][0]
                            self.y_event = sol5.y_events[0][0]
                            if( self.y_event[1]<5):
                                self.e7.succeed()
                                self.flag =7
                               
                                self.t_start = self.t_event
                            else:
                                self.e6.succeed()
                            print(f"mode 5 Événement détecté à t = {self.t_event:.2f}, x = {self.y_event[0]:.2f}, z = {self.y_event[1]:.2f}, theta = {self.y_event[2]:.2f}")
                            self.condition_initial = self.y_event
                            self.t_start = self.t_event
                        else:
                            print("Événement non détecté dans mode 5")

                case "6":
                    if self.flag == 7:
                        self.flag=8
                        self.condition_initial = self.y_event[0], self.y_event[1], 0, 0, 0, 0
                        self.t_mode_6 = np.linspace(self.t_start, self.t_end, self.resolution)
                        sol6 = solve_ivp(
                            self.drone_dynamics,
                            (self.t_start, self.t_end),
                            self.condition_initial,
                            method="RK45",
                            t_eval=self.t_mode_6,
                            args=(self.params,),
                            
                        )
                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = sol6.y

                        self.x_data = np.concatenate((self.x_data, self.x))
                        self.z_data = np.concatenate((self.z_data, self.z))
                        self.theta_data = np.concatenate((self.theta_data, self.theta))
                        self.Vx_data = np.concatenate((self.Vx_data, self.Vx))
                        self.Vz_data = np.concatenate((self.Vz_data, self.Vz))
                        self.Vtheta_data = np.concatenate((self.Vtheta_data, self.Vtheta))

                        self.t_mode_6_data = sol6.t
                        self.temps = np.concatenate((self.temps, self.t_mode_6_data))
                            

                case "7":

                    if self.flag == 9:
                        self.flag=0
                       
                        self.t_mode_7 = np.linspace(self.t_start, self.t_end, self.resolution)
                        sol7 = solve_ivp(
                            self.drone_dynamics,
                            (self.t_start, self.t_end),
                            self.condition_initial,
                            method="RK45",
                            t_eval=self.t_mode_7,
                            args=(self.params,),
                             events=self.z_egal_40,
                            dense_output=True,
                            
                        )
                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = sol7.y

                        self.x_data = np.concatenate((self.x_data, self.x))
                        self.z_data = np.concatenate((self.z_data, self.z))
                        self.theta_data = np.concatenate((self.theta_data, self.theta))
                        self.Vx_data = np.concatenate((self.Vx_data, self.Vx))
                        self.Vz_data = np.concatenate((self.Vz_data, self.Vz))
                        self.Vtheta_data = np.concatenate((self.Vtheta_data, self.Vtheta))

                        self.t_mode_7_data = sol7.t
                        self.temps = np.concatenate((self.temps, self.t_mode_7_data))
                        if len(sol7.t_events) > 0 and sol7.t_events[0].size > 0:
                            self.t_event = sol7.t_events[0][0]
                            self.y_event = sol7.y_events[0][0]
                            self.e9.succeed()
                            print(f"mode 7 Événement détecté à t = {self.t_event:.2f}, x = {self.y_event[0]:.2f}, z = {self.y_event[1]:.2f}, theta = {self.y_event[2]:.2f}")
                            self.condition_initial = self.y_event[0],self.y_event[1],0,0,0,0
                            self.t_start = self.t_event
                        else:
                            print("Événement non détecté dans mode 7")
                    
                    
            self.compteur += 1
            yield self.env.timeout(1)

    def automate(self):
        while True:
            match self.mode:
                case "0":
                    if self.flag2 == 0:
                        self.flag2 = 1
                    yield self.e0
                    self.e0 = self.env.event()
                    self.mode = "1"
                case "1":
                    if self.flag2 == 1:
                        pass
                    yield self.e1 | self.e3
                    if self.e1.triggered:
                        self.flag2 = 2
                        self.e1 = self.env.event()
                        self.mode = "2"
                    elif self.e3.triggered:
                        self.flag2 = 3
                        self.e3 = self.env.event()
                        self.mode = "3"
                case "2":
                    if self.flag2 == 2:
                        pass
                    yield self.e2 | self.e3
                    if self.e2.triggered:
                        self.flag2 = 1
                        self.e2 = self.env.event()
                        self.mode = "1"
                    elif self.e3.triggered:
                        self.flag2 = 3
                        self.e3 = self.env.event()
                        self.mode = "3"
                case "3":
                    if self.flag2 == 3:
                        
                        pass
                    yield self.e4|self.e8
                    if self.e4.triggered:
                         self.flag2 = 4
                         self.e5= self.env.event()
                         self.mode = "4" 
                    elif self.e8.triggered:
                         self.flag2 = 7
                         self.e8 = self.env.event()
                         self.mode = "7" 
                case "4":
                    if self.flag2 == 4:
                        self.flag2 = 5
                        pass
                    yield self.e5|self.e7
                    if self.e5.triggered:
                         self.flag2 = 5
                         self.e5= self.env.event()
                         self.mode = "5" 
                    elif self.e7.triggered:
                         self.flag2 = 6
                         self.e7 = self.env.event()
                         self.mode = "6" 

                case "5":
                    if self.flag2 == 5:
                        
                        pass
                    yield self.e6 | self.e7
                    if self.e6.triggered:
                        self.flag2 = 4
                        self.e6 = self.env.event()
                        self.mode = "4" 
                    elif self.e7.triggered:
                        self.flag2 = 6
                        self.e7 = self.env.event()
                        self.mode = "6" 

                case "6":
                    if self.flag2 == 6:
                       
                        pass


                case "7":
                    if self.flag2 == 7:
                       
                        pass

                    yield self.e9
                    self.e9 = self.env.event()
                    self.mode = "1" 
                    
                   
                   

            yield self.env.timeout(1)


def plot_all_results(t, x, z, theta, Vx, Vz, Vtheta):
    # Convertir en arrays NumPy si nécessaire
    t = np.asarray(t)
    x = np.asarray(x)
    z = np.asarray(z)
    theta = np.asarray(theta)
    Vx = np.asarray(Vx)
    Vz = np.asarray(Vz)
    Vtheta = np.asarray(Vtheta)

    # Créer des plots individuels
    # Position Horizontale
    plt.figure(figsize=(10, 6))
    plt.plot(t, x)
    plt.title("Position Horizontale")
    plt.xlabel("t (s)")
    plt.ylabel("x (m)")
    plt.grid()
    plt.savefig("position_horizontale.png")
    plt.show()

    # Vitesse Horizontale
    plt.figure(figsize=(10, 6))
    plt.plot(t, Vx)
    plt.title("Vitesse Horizontale")
    plt.xlabel("t (s)")
    plt.ylabel("Vx (m/s)")
    plt.grid()
    plt.savefig("vitesse_horizontale.png")
    plt.show()

    # Position Verticale
    plt.figure(figsize=(10, 6))
    plt.plot(t, z)
    plt.title("Position Verticale")
    plt.xlabel("t (s)")
    plt.ylabel("z (m)")
    plt.grid()
    plt.savefig("position_verticale.png")
    plt.show()

    # Vitesse Verticale
    plt.figure(figsize=(10, 6))
    plt.plot(t, Vz)
    plt.title("Vitesse Verticale")
    plt.xlabel("t (s)")
    plt.ylabel("Vz (m/s)")
    plt.grid()
    plt.savefig("vitesse_verticale.png")
    plt.show()

    # Angle θ
    plt.figure(figsize=(10, 6))
    plt.plot(t, theta)
    plt.title("Angle θ")
    plt.xlabel("t (s)")
    plt.ylabel("θ (rad)")
    plt.grid()
    plt.savefig("angle_theta.png")
    plt.show()

    # Vitesse Angulaire
    plt.figure(figsize=(10, 6))
    plt.plot(t, Vtheta)
    plt.title("Vitesse Angulaire")
    plt.xlabel("t (s)")
    plt.ylabel("Vθ (rad/s)")
    plt.grid()
    plt.savefig("vitesse_angulaire.png")
    plt.show()





def main():
    m = 0.01  # kg
    L = 0.086  # m (86 mm, ajusté pour un drone plus réaliste)
    g = 9.81  # m/s^2
    I = (1 / 12) * m * L**2  # moment d'inertie (kg*m^2)
    params = {"m": m, "L": L, "g": g, "I": I}

    env = Environment()
    machine = Hybrid_SYSTEM(env, params)

    env.process(machine.dynamique_continu())
    env.process(machine.automate())

    env.run(until=machine.resolution)

    if len(machine.temps) > 0 and len(machine.x_data) > 0:
        print("Simulation terminée avec succès")
        plot_all_results(machine.temps, machine.x_data, machine.z_data, machine.theta_data, machine.Vx_data, machine.Vz_data, machine.Vtheta_data)
        plt.plot(machine.x_data, machine.z_data)
        plt.title("Altitude du drone en fonction du temps")
        plt.xlabel("x (m)")
        plt.ylabel("Altitude (m)")
        plt.grid()
        plt.savefig("altitude_vs_temps.png")
        plt.show()
       
    else:
        print("Aucune donnee generee par la simulation")
#----------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
