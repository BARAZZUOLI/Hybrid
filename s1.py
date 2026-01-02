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
        self.t_end = 100
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

        # Constantes système

        self.params = params

        self.F1 = 0
        self.F2 = 0

        self.flag = 0
        self.flag2 = 0
        self.flag3=0

        self.F_g = self.params["m"] * self.params["g"]
        self.force_contraire=0

    """
    Les évement suivant permmettent de stpopper l'intégration numérique lorsque la position z atteint 50m ou 60m,
    afin de changer le mode du système hybride.
    """

  

    """
    Les évement suivant permmettent de stpopper l'intégration numérique lorsque la position x atteint 250m ou 300m,
    afin de changer le mode du système hybride.
    """

    """
    Les évement suivant permmettent de stpopper l'intégration numérique lorsque la position theta atteint  rad ou rad,
    afin de changer le mode du système hybride.
    """

    def theta_sup_pos_0_01_rad(self, t, y, params):
        return y[2] + 0.01

    theta_sup_pos_0_01_rad.terminal = True
    theta_sup_pos_0_01_rad.direction = 1

    def theta_inf_neg_0_01_rad(self, t, y, params):
        return y[2] + 0.01

    theta_inf_neg_0_01_rad.terminal = True
    theta_inf_neg_0_01_rad.direction = -1


    def theta_sup_pos_1_rad(self, t, y, params):
        return y[2] + 3*pi/4

    theta_sup_pos_1_rad.terminal = True
    theta_sup_pos_1_rad.direction = 1
    
    def theta_inf_neg_1_rad(self, t, y, params):
        return y[2] + 3*pi/4

    theta_inf_neg_1_rad.terminal = True
    theta_inf_neg_1_rad.direction = -1



    def theta_egal_pi_rad(self, t, y, params):
        return y[2] + pi/2

    theta_egal_pi_rad.terminal = True
    theta_egal_pi_rad.direction = -1
    





  

    """La Consigne U en fonction du mode de fonctionement de l'automate hybride"""

    def u(self, t):
        match self.mode:

            case "0":

                F1 = self.F_g
                F2 = self.F_g

            case "1":

                F1 =  t
                F2 =  t - 0.001

            case "2":

               
                F1 =  t- 0.001
                F2 =  t 

            case "3":
                F1 =  10.1
                F2 =  10
               

            case "4":

                 F1 =  t
                 F2 =  t- 0.001 

                
            case "5":

                 F1 =  t- 0.001
                 F2 =  t 

        
        return [F1, F2]

    """Dynamique du système source: https://github.com/fpelogia/drone-simulation.git """

    def drone_dynamics(self, t, state, params):

        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = state

        self.F1, self.F2 = self.u(t)

        self.m = params["m"]
        self.I = params["I"]
        self.L = params["L"]
        self.g = params["g"]

        self.Ax = -((self.F1 + self.F2) * np.sin(self.theta)) / self.m
        self.Az = ((self.F1 + self.F2) * np.cos(self.theta)) / self.m - self.g
        self.Atheta = ((self.F2 - self.F1) * self.L) / (2 * self.I)

        dot_state = [self.Vx, self.Vz, self.Vtheta, self.Ax, self.Az, self.Atheta]

        return dot_state

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
                            self.t_mode_1_data = sol1.t
                            self.temps = np.concatenate((self.temps, self.t_mode_1_data))

                            """Vérification si événement a eu lieu """

                            if sol1.t_events[0].size > 0:
                                self.t_event = sol1.t_events[0][0]
                                self.y_event = sol1.y_events[0][0]

                                if self.y_event[1] >= 500:
                                    self.e3.succeed()
                                else:
                                    self.e1.succeed()
                                

                                print(
                                f"mode 1 Événement détecté à t = {self.t_event:.2f}, theta = {self.y_event[2]:.2f}, z = {self.y_event[1]:.2f}"
                                )

                                # État initial pour le mode 2

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
                        self.t_mode_2_data = sol2.t
                        self.temps = np.concatenate((self.temps, self.t_mode_2_data))
                        """Vérification si événement a eu lieu """

                        if sol2.t_events[0].size > 0:
                            
                            self.t_event = sol2.t_events[0][0]
                            self.y_event = sol2.y_events[0][0]
                            if self.y_event[1] >= 500:
                                    self.e3.succeed()
                            else:
                                    self.e2.succeed()

                            print(
                                f"mode 2 Événement détecté à t = {self.t_event:.2f}, theta = {self.y_event[2]:.2f}, z = {self.y_event[1]:.2f}"
                            )

                            # État initial pour le mode 2

                            self.condition_initial = self.y_event
                            self.t_start = self.t_event

                        else:
                            print("Événement non détecté dans mode 2")

                case "3":
                   
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
                           events=self.theta_egal_pi_rad,  # Événement pour arrêter l'intégration numérique
                           dense_output=True,  # S'il faut calculer une solution continue. La valeur par défaut est False
                           
                        )

                        """mise à jour  des états aprés integration numérique """

                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = (
                            sol3.y
                        )

                        """stockage"""

                        self.x_data = np.concatenate((self.x_data, self.x))
                        self.z_data = np.concatenate((self.z_data, self.z))
                        self.theta_data = np.concatenate((self.theta_data, self.theta))
                        self.t_mode_3_data = sol3.t
                        self.temps = np.concatenate((self.temps, self.t_mode_3_data))


                        """Vérification si événement a eu lieu """
#                       
                        if sol3.t_events[0].size > 0:

                            self.e4.succeed()
                            self.t_event = sol3.t_events[0][0]
                            self.y_event = sol3.y_events[0][0]
                            print(
                                f"mode 3 Événement détecté à t = {self.t_event:.2f}, theta = {self.y_event[2]:.2f}, z = {self.y_event[1]:.2f}"
                            )

                            # État initial pour le mode 2

                            self.condition_initial = self.y_event
                            self.t_start = self.t_event

                        else:
                            print("Événement non détecté dans mode 3")
                case "4":

                    if (
                        self.flag == 1
                    ):  # |---> Pour éviter de faire intégration numerique plusieur fois
                        self.flag = 0  # |

                        self.t_mode_4= np.linspace(
                            self.t_start, self.t_end, self.resolution
                        )

                        sol4 = solve_ivp(
                            self.drone_dynamics,  # Dynamique du système
                            (
                                self.t_start,
                                self.t_end,
                            ),  # intervalle de temps intégration numérique
                            self.condition_initial,  # états du système (xdotdot,zdotdot,thetadotdot)
                            method="RK45",  # Méthode d'intégration
                            t_eval=self.t_mode_4,  # tab-->temps
                            args=(self.params,),  # constante du système
                            events=self.theta_inf_neg_1_rad,  # Événement pour arrêter l'intégration numérique
                            dense_output=True,  # S'il faut calculer une solution continue. La valeur par défaut est False
                           
                        )

                        """mise à jour  des états aprés integration numérique """

                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = (
                            sol4.y
                        )

                        """stockage"""

                        self.x_data = np.concatenate((self.x_data, self.x))
                        self.z_data = np.concatenate((self.z_data, self.z))
                        self.theta_data = np.concatenate((self.theta_data, self.theta))
                        self.t_mode_4_data = sol4.t
                        self.temps = np.concatenate((self.temps, self.t_mode_4_data))


                        """Vérification si événement a eu lieu """

                        if sol4.t_events[0].size > 0:

                            self.e5.succeed()
                            self.t_event = sol4.t_events[0][0]
                            self.y_event = sol4.y_events[0][0]
                            print(
                                f"mode 4 Événement détecté à t = {self.t_event:.2f}, theta = {self.y_event[2]:.2f}, z = {self.y_event[1]:.2f}"
                            )

                            # État initial pour le mode 2

                            self.condition_initial = self.y_event
                            self.t_start = self.t_event

                        else:
                            print("Événement non détecté dans mode 4")

                case "5":

                    if (
                        self.flag == 0
                    ):  # |---> Pour éviter de faire intégration numerique plusieur fois
                        self.flag = 0  # |

                        self.t_mode_5= np.linspace(
                            self.t_start, self.t_end, self.resolution
                        )

                        sol5 = solve_ivp(
                            self.drone_dynamics,  # Dynamique du système
                            (
                                self.t_start,
                                self.t_end,
                            ),  # intervalle de temps intégration numérique
                            self.condition_initial,  # états du système (xdotdot,zdotdot,thetadotdot)
                            method="RK45",  # Méthode d'intégration
                            t_eval=self.t_mode_5,  # tab-->temps
                            args=(self.params,),  # constante du système
                            events=self.theta_sup_pos_1_rad,  # Événement pour arrêter l'intégration numérique
                            dense_output=True,  # S'il faut calculer une solution continue. La valeur par défaut est False
                           
                        )

                        """mise à jour  des états aprés integration numérique """

                        self.x, self.z, self.theta, self.Vx, self.Vz, self.Vtheta = (
                            sol5.y
                        )

                        """stockage"""

                        self.x_data = np.concatenate((self.x_data, self.x))
                        self.z_data = np.concatenate((self.z_data, self.z))
                        self.theta_data = np.concatenate((self.theta_data, self.theta))
                        self.t_mode_5_data = sol5.t
                        self.temps = np.concatenate((self.temps, self.t_mode_5_data))


                        """Vérification si événement a eu lieu """

                        if sol5.t_events[0].size > 0:

                            self.e6.succeed()
                            self.t_event = sol5.t_events[0][0]
                            self.y_event = sol5.y_events[0][0]
                            print(
                               f"mode 5 Événement détecté à t = {self.t_event:.2f}, theta = {self.y_event[2]:.2f}, z = {self.y_event[1]:.2f}"
                            )

                            # État initial pour le mode 2

                            self.condition_initial = self.y_event
                            self.t_start = self.t_event

                        else:
                            print("Événement non détecté dans mode 4")
               

                    

            self.compteur += 1
            yield self.env.timeout(1)

    def automate(self):
        while True:
            match self.mode:

                case "0":

                    """Mode 0"""

                    if (
                        self.flag2 == 0
                    ):  # |--> Pour éviter de print le mode 0 plusieur fois
                        self.flag2 = 1  # |

                       # print("--> mode[0]<--")

                    """Attente de l'évenement e1"""

                    yield self.e0
                    self.mode = "1"

                case "1":

                    """Mode 1"""

                    if (
                        self.flag2 == 1
                    ):  # |--> Pour éviter de print le mode 0 plusieur fois
                      

                        #print("--> mode[1]<--")

                     """Attente de l'évenement e1"""

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

                    """Mode 2"""

                    if self.flag2 == 2:
                        

                        #print("--> mode[2]<--")

                     """Attente de l'évenement e2 ou e3"""

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

                    """Mode 3"""

                    if self.flag2 == 3:
                        self.flag2 = 4
                        print("--> mode[3]<--")

                    """Attente de l'évenement e4"""

                    yield self.e4
                    self.e4 = self.env.event()
                    self.mode = "4"

                case "4":

                    """Mode 4"""

                    if self.flag2 == 4:
                        self.flag2 = 3

                        print("--> mode[4]<--")

                    """Attente de l'évenement e5"""

                    yield self.e5
                    self.e5 = self.env.event()
                    self.mode = "5"


                case "5":

                    """Mode 5"""

                    if self.flag2 == 4:
                        self.flag2 = 3

                        print("--> mode[4]<--")

                    """Attente de l'évenement e6"""

                    yield self.e6
                    self.e6 = self.env.event()
                    self.mode = "4"
                    

                    

                  

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


def plot_trajectory(x, y, theta, t_end, params):
    """Animation de la trajectoire"""
    if len(x) == 0 or len(y) == 0:
        print("Aucune donnÃ©e pour l'animation")
        return None

    # unpack parameters
    L = params["L"]

    fig, ax = plt.subplots()
    (traj_plot,) = ax.plot([], [])

    # CrÃ©er le patch du drone
    drone_width = L
    drone_height = 0.2*50
    patch = plt.Rectangle(
        (0, 0), drone_width, drone_height, angle=0, color="r", alpha=0.8
    )
    ax.add_patch(patch)

    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Trajectory")

    # DÃ©finir les limites
    if len(x) > 0:
        margin = 2
        plt.xlim(min(x) - margin, max(x) + margin)
        plt.ylim(0, max(y) + margin)

    # animation
    def update(frame):
        if frame >= len(x):
            return traj_plot, patch

        # Mettre Ã  jour la trajectoire
        traj_plot.set_data(x[: frame + 1], y[: frame + 1])

        # Mettre Ã  jour le drone
        patch.set_xy((x[frame] - L / 2, y[frame] - drone_height / 2))
        patch.set_angle(theta[frame])

        return traj_plot, patch

    # Calculer le nombre de frames (limitÃ© pour performance)
    n_frames = min(len(x), 200)

    ani = animation.FuncAnimation(
        fig=fig, func=update, frames=n_frames, interval=50, repeat=False
    )

    try:
        ani.save("trajectory.gif", writer="pillow", fps=20)
        print("Animation sauvegardÃ©e dans 'trajectory.gif'")
    except:
        print("Erreur lors de la sauvegarde de l'animation")

    plt.show()
    return ani


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
        print(f"ok")
        plot_results(machine.temps, machine.x_data, machine.z_data, machine.theta_data)
        plot_trajectory(
            machine.x_data, machine.z_data, machine.theta_data, machine.t_end, params
        )
    else:
        print("Aucune donnÃ©e gÃ©nÃ©rÃ©e par la simulation")


if __name__ == "__main__":
    main()
