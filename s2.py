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


class Hybrid_SYSTEM:

    def __init__(self, env, params):

        # Temps

        self.t_start = 0
        self.t_end = 4
        self.resolution = 1000
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

        # Constantes système

        self.params = params

        self.F1 = 0
        self.F2 = 0
        self.F3 = 0
        self.F4 = 0

        self.flag = 0
        self.flag2 = 0
        self.flag3=0

        self.F_g = (self.params["m"] * self.params["g"]/2)  # Force de gravité divisée par 2 pour chaque moteur
        


    def u(self, t):
    
        match self.mode:
            case "0":
                F1 = self.F_g
                F2 = self.F_g
                F3=0
                F4=0
            case "1":
                F1 = t 
                F2 = t - 0.001
                F3=0
                F4=0
            case "2":
                F1 = t - 0.001
                F2 = t
                F3=0
                F4=0
            case "3":
             
              
                    F1 =0
                    F2 = 0
                    F3 = 0
                    F4 = 0

        return [F1, F2, F3, F4] # type: ignore

    """Dynamique du système source: https://github.com/fpelogia/drone-simulation.git """

    def drone_dynamics(self, t, state, params):

        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = state

        self.F1, self.F2, self.F3, self.F4 = self.u(t)

        self.m = params["m"]
        self.I = params["I"]
        self.L = params["L"]
        self.g = params["g"]

        self.Ax = -(((self.F1 + self.F2) * np.sin(self.theta)) / self.m )+(self.F3-self.F4)
        self.Az = ((self.F1 + self.F2) * np.cos(self.theta)) / self.m - self.g
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

    """Dynamique continue du système hybride """

    def dynamique_continu(self):
        while True:
            match self.mode:
             case "0":

                    """Mode 0"""
                    self.t_start = self.compteur * (1 / self.resolution)
                    if self.compteur == (self.resolution-(self.resolution-10)):
                        self.e0.succeed()

             case "1":

                        
                        
                        if (
                            self.flag == 0
                        ):  # |---> Pour éviter de faire intégration numerique plusieur fois
                            self.flag = 1  # |

                            self.t_mode_1 = np.linspace(
                                self.t_start, self.t_end, self.resolution
                            )

                            """Integration numérique """

                            sol1 = solve_ivp(
                                self.drone_dynamics,  # Dynamique du système
                                (
                                    self.t_start,
                                    self.t_end,
                                ),  # intervalle de temps intégration numérique
                                self.condition_initial,  # états du système (xdotdot,zdotdot,thetadotdot)
                                method="RK45",  # Méthode d'intégration
                                t_eval=self.t_mode_1,  # tab-->temps
                                args=(self.params,),  # constante du système
                                events=self.theta_inf_neg_0_01_rad,  # Événement pour arrêter l'intégration numérique
                                dense_output=True,  # S'il faut calculer une solution continue. La valeur par défaut est False
                            )

                            """Mise à jour  des états aprés integration numérique """

                            self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = (
                                sol1.y
                            )

                            """Stockage"""

                            self.x_data = np.concatenate((self.x_data, self.x))
                            self.z_data = np.concatenate((self.z_data, self.z))
                            self.theta_data = np.concatenate((self.theta_data, self.theta))
                            self.Vx_data = np.concatenate((self.Vx_data, self.Vx))
                            self.Vz_data = np.concatenate((self.Vz_data, self.Vz))
                            self.Vtheta_data = np.concatenate((self.Vtheta_data, self.Vtheta))
                            self.t_mode_1_data = sol1.t
                            self.temps = np.concatenate((self.temps, self.t_mode_1_data))

                            """Vérification si événement a eu lieu """

                            if sol1.t_events[0].size > 0:
                                self.t_event = sol1.t_events[0][0]
                                self.y_event = sol1.y_events[0][0]

                                if self.y_event[1]:
                                    self.e3.succeed()
                                else:
                                    self.e1.succeed()
                               
                                
                                print(
                                f"mode 1 Événement détecté à t = {self.t_event:.2f}, theta = {self.y_event[2]:.2f}, z = {self.y_event[1]:.2f}"
                                )

                                self.condition_initial = self.y_event
                                self.t_start = self.t_event

                            else:
                                print("Événement non détecté dans mode 1")

             case "2":

                    """Mode 2"""
                  
                    if (
                        self.flag == 1 
                    ):  # |---> Pour éviter de faire intégration numerique plusieur fois
                        self.flag = 0  # |

                        self.t_mode_2 = np.linspace(
                            self.t_start, self.t_end, self.resolution
                        )

                        sol2 = solve_ivp(
                            self.drone_dynamics,  # Dynamique du système
                            (
                                self.t_start,
                                self.t_end,
                            ),  # intervalle de temps intégration numérique
                            self.condition_initial,  # états du système (xdotdot,zdotdot,thetadotdot)
                            method="RK45",  # Méthode d'intégration
                            t_eval=self.t_mode_2,  # tab-->temps
                            args=(self.params,),  # constante du système
                            events=self.theta_sup_pos_0_01_rad,  # Événement pour arrêter l'intégration numérique
                            dense_output=True,  # S'il faut calculer une solution continue. La valeur par défaut est False
                        )

                        """mise à jour  des états aprés integration numérique """

                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = (
                            sol2.y
                        )

                        """stockage"""

                        self.x_data = np.concatenate((self.x_data, self.x))
                        self.z_data = np.concatenate((self.z_data, self.z))
                        self.theta_data = np.concatenate((self.theta_data, self.theta))
                        self.Vx_data = np.concatenate((self.Vx_data, self.Vx))
                        self.Vz_data = np.concatenate((self.Vz_data, self.Vz))
                        self.Vtheta_data = np.concatenate((self.Vtheta_data, self.Vtheta))
                        self.t_mode_2_data = sol2.t
                        self.temps = np.concatenate((self.temps, self.t_mode_2_data))
                        """Vérification si événement a eu lieu """

                        if sol2.t_events[0].size > 0:
                            
                            self.t_event = sol2.t_events[0][0]
                            self.y_event = sol2.y_events[0][0]
                           
                            if self.y_event[1]>500:
                                self.e3.succeed()
                            else:
                                self.e2.succeed()

                            print(
                                f"mode 2 Événement détecté à t = {self.t_event:.2f}, theta = {self.y_event[2]:.2f}, z = {self.y_event[1]:.2f}"
                            )


                            self.condition_initial = self.y_event
                            self.t_start = self.t_event

                        else:
                            print("Événement non détecté dans mode 2")
             case "3":
                    """Mode 3"""
                    if (
                        self.flag == 0 or self.flag==1
                    ):  # |---> Pour éviter de faire intégration numerique plusieur fois
                        self.flag = 1  # |


                       

                        self.t_mode_3= np.linspace(
                            self.t_start, self.t_end, self.resolution
                        )

                        sol3 = solve_ivp(
                            self.drone_dynamics,  # Dynamique du système
                            (
                                self.t_start,
                                self.t_end,
                            ),  # intervalle de temps intégration numérique
                            self.condition_initial,  # états du système (xdotdot,zdotdot,thetadotdot)
                            method="RK45",  # Méthode d'intégration
                            t_eval=self.t_mode_3,  # tab-->temps
                            args=(self.params,),  # constante du système
                           
                        )

                        """mise à jour  des états aprés integration numérique """

                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = (
                            sol3.y
                        )

                        """stockage"""

                        self.x_data = np.concatenate((self.x_data, self.x))
                        self.z_data = np.concatenate((self.z_data, self.z))
                        self.theta_data = np.concatenate((self.theta_data, self.theta))
                        self.Vx_data = np.concatenate((self.Vx_data, self.Vx))
                        self.Vz_data = np.concatenate((self.Vz_data, self.Vz))
                        self.Vtheta_data = np.concatenate((self.Vtheta_data, self.Vtheta))
                        self.t_mode_3_data = sol3.t
                        self.temps = np.concatenate((self.temps, self.t_mode_3_data))


            
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
                        print("--> mode[3]<--")
                    
            yield self.env.timeout(1)


def plot_all_results(t, x, z, theta, Vx, Vz, Vtheta):
    fig, axs = plt.subplots(3, 2, figsize=(10, 15))
    plt.subplots_adjust(hspace=0.5, wspace=0.4)

    # Graphique de la position horizontale et vitesse horizontale
    axs[0, 0].plot(t, x)
    axs[0, 0].set_title("Position Horizontale")
    axs[0, 0].set_xlabel("t (s)")
    axs[0, 0].set_ylabel("x (m)")

    axs[1, 0].plot(t, Vx)
    axs[1, 0].set_title("Vitesse Horizontale")
    axs[1, 0].set_xlabel("t (s)")
    axs[1, 0].set_ylabel("Vx (m/s)")

    # Graphique de la position verticale et vitesse verticale
    axs[0, 1].plot(t, z)
    axs[0, 1].set_title("Position Verticale")
    axs[0, 1].set_xlabel("t (s)")
    axs[0, 1].set_ylabel("z (m)")

    axs[1, 1].plot(t, Vz)
    axs[1, 1].set_title("Vitesse Verticale")
    axs[1, 1].set_xlabel("t (s)")
    axs[1, 1].set_ylabel("Vz (m/s)")

    # Graphique de l'angle et vitesse angulaire
    axs[2, 0].plot(t, theta)
    axs[2, 0].set_title("Angle θ")
    axs[2, 0].set_xlabel("t (s)")
    axs[2, 0].set_ylabel("θ (rad)")

    axs[2, 1].plot(t, Vtheta)
    axs[2, 1].set_title("Vitesse Angulaire")
    axs[2, 1].set_xlabel("t (s)")
    axs[2, 1].set_ylabel("Vθ (rad/s)")

    for ax in axs.flat:
        ax.grid()

    plt.savefig("results_combined.png")
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
