"""
drone_core.py
-------------
Bibliothèque de simulation physique, d'intégration et de contrôle PID pour drone.
"""

import matplotlib.pyplot as plt
import numpy as np

import asservissement as asserv
import dynamique as dyn
import fonctions as fct
import Lecture_data as ld

SEUIL_VIT_ANGULAIRE_MAX = 500.0  # °/s


class DroneSimulation:
    def __init__(self, config_path="Donnees_drone.txt"):
        self.config_path = config_path
        self.g = 9.81

        # Chargement des paramètres
        dic = ld.lire_configuration(self.config_path, par_section=True)
        self.Masse = dic["DONNEE DRONE"]["Masse"]
        self.Longueur = dic["DONNEE DRONE"]["Longueur"]
        self.Largeur = dic["DONNEE DRONE"]["Largeur"]
        self.Hauteur = dic["DONNEE DRONE"]["Hauteur"]
        self.L = dic["DONNEE DRONE"]["Moteur_L"]
        self.l = dic["DONNEE DRONE"]["Moteur_l"]
        
        self.barycentre = np.array(dic["DONNEE DRONE"]["Barycentre"])

        self.hauteur_helice = dic["DONNEE DRONE"]["Hauteur_helice"]

        print(self.barycentre[0], self.barycentre[1], self.barycentre[2])
        print(self.hauteur_helice)

        self.Mat_Inertie = np.array(dic["DONNEE DRONE"]["Matrice_Inertie"])
        self.FORCE_MAX_MOTEUR = 3.85  # Force Max 3.85 N

        self.delta_t = dic["SIMULATION"]["delta_t"]
        self.k_frot = dic["SIMULATION"]["k_frot"]
        self.angle_max = 45

        self.tau_esc = dic["SIMULATION"]["tau_esc"]  # Constante de temps des ESC/moteurs

        # Gains PID modifiables
        self.pid_gains = {
            "roll":  {"kp": 20.0, "ki": 4.0, "kd": 16.0},
            "pitch": {"kp": 14.0123754, "ki": 1.0, "kd": 10.0},
            "alt":   {"kp": 49.898034, "ki": 0.0082477988, "kd": 1.34603797}
        }

        # Paramètres de la batterie (3S LiPo : 12.6V max, 11.1V nominal, 10.5V min)
        self.capacite_batterie_Ah = 2.3  # 2300 mAh
        self.taux_decharge_C = 3.0       
        self.courant_max_batterie = self.capacite_batterie_Ah * self.taux_decharge_C  # 6.9 A
        
        self.tension_max = 12.6          
        self.tension_min = 10.5          
        self.tension_nominal = 11.1      
        self.resistance_interne = 0.15   
        
        self.capacite_restante_Ah = self.capacite_batterie_Ah
        self.tension_actuelle = self.tension_max
        self.soc_batterie = 100.0        
        self.batterie_critique = False

        self.Poids_global = np.array([0.0, 0.0, -self.Masse * self.g])
        
        self.poids_par_moteur = (self.Masse * self.g) / 4.0
        self.esc_equilibre = fct.calcul_force2commande(self.poids_par_moteur)

        self.hist_roll = []
        self.hist_pitch = []

        self.reset()

    def reset(self):
        """Réinitialise l'état physique complet."""
        self.pos = np.array([0.0, 0.0, 0.0])
        self.pos_actuel = np.copy(self.pos)
        self.angle = np.array([0.0, 0.0, 0.0])
        self.angle_actuel = np.copy(self.angle)

        self.t = 0.0
        self.drone_demarre = False
        self.POWER = 0.0
        self.Stab_Alt = True

        self.f_A = self.f_B = self.f_C = self.f_D = 0.0
        self.esc_A = self.esc_B = self.esc_C = self.esc_D = 1000.0
        
        self.vitesse = np.array([0.0, 0.0, 0.0])
        self.vitesse_angulaire_deg = np.array([0.0, 0.0, 0.0])
        self.norme_vitesse = 0.0
        self.poussee_totale = 0.0
        self.alerte_vitesse_angulaire = False

        self.hist_roll.clear()
        self.hist_pitch.clear()

    def toggle_motors(self):
        self.drone_demarre = not self.drone_demarre

    def step(self, stick_gx=0.0, stick_gy=0.0, stick_dy=0.0):
        """Pas d'intégration physique."""
        Poids_Moteur = self.Masse * self.g / (4 * np.cos(self.angle[0]) * np.cos(self.angle[1]))
        esc_Moteur = fct.calcul_force2commande(Poids_Moteur)
        esc_throttle = esc_Moteur

        self.POWER += stick_dy**3 * 10
        self.Stab_Alt = abs(stick_dy) < 0.03
        if self.Stab_Alt:
            self.POWER = 0.0

        if self.drone_demarre:
            esc_com = fct.calcul_commande2controleur(self.POWER)
            esc_throttle = np.clip(esc_com + esc_Moteur, 1000, 1800)
            cmd_angle_roll = stick_gx * self.angle_max #* np.pi / 180
            cmd_angle_pitch = -stick_gy * self.angle_max #* np.pi / 180
        else:
            cmd_angle_roll = 0.0
            cmd_angle_pitch = 0.0

        vit = (self.pos - self.pos_actuel) / self.delta_t

        # Correction PID
        esc_corr_A, esc_corr_B, esc_corr_C, esc_corr_D = asserv.calcul_correction_PID(
            np.degrees(self.angle_actuel),
            vit[2],
            altitude_corr=self.Stab_Alt,
            angle_cible=np.array([cmd_angle_roll, cmd_angle_pitch, 0.0]),
            Kp_roll=self.pid_gains["roll"]["kp"], Kp_pitch=self.pid_gains["pitch"]["kp"], Kp_yaw=0.1, Kp_alt=self.pid_gains["alt"]["kp"],
            Ki_roll=self.pid_gains["roll"]["ki"], Ki_pitch=self.pid_gains["pitch"]["ki"], Ki_yaw=0.005, Ki_alt=self.pid_gains["alt"]["ki"],
            Kd_roll=self.pid_gains["roll"]["kd"], Kd_pitch=self.pid_gains["pitch"]["kd"], Kd_yaw=0.02, Kd_alt=self.pid_gains["alt"]["kd"],
            delta_t=self.delta_t,
            reset_pid=not self.drone_demarre,
        )

        if self.drone_demarre:
            self.esc_A = float(np.clip(esc_throttle + esc_corr_A, 0, 2000))
            self.esc_B = float(np.clip(esc_throttle + esc_corr_B, 0, 2000))
            self.esc_C = float(np.clip(esc_throttle + esc_corr_C, 0, 2000))
            self.esc_D = float(np.clip(esc_throttle + esc_corr_D, 0, 2000))

            alpha = self.delta_t / (self.tau_esc + self.delta_t)
            # self.esc_A += alpha * (target_esc_A - self.esc_A)
            # self.esc_B += alpha * (target_esc_B - self.esc_B)
            # self.esc_C += alpha * (target_esc_C - self.esc_C)
            # self.esc_D += alpha * (target_esc_D - self.esc_D)

            self.esc_A = float(np.clip(1000.0, self.esc_A, 1800.0))
            self.esc_B = float(np.clip(1000.0, self.esc_B, 1800.0))
            self.esc_C = float(np.clip(1000.0, self.esc_C, 1800.0))
            self.esc_D = float(np.clip(1000.0, self.esc_D, 1800.0))

            self.f_A = float(np.clip(fct.calcul_controleur2force(self.esc_A), 0.0, self.FORCE_MAX_MOTEUR))
            self.f_B = float(np.clip(fct.calcul_controleur2force(self.esc_B), 0.0, self.FORCE_MAX_MOTEUR))
            self.f_C = float(np.clip(fct.calcul_controleur2force(self.esc_C), 0.0, self.FORCE_MAX_MOTEUR))
            self.f_D = float(np.clip(fct.calcul_controleur2force(self.esc_D), 0.0, self.FORCE_MAX_MOTEUR))

            # # Gestion batterie
            # courant_moteur_A = max(0.0, (self.esc_A - 1000) / 800.0 * 2.0)
            # courant_moteur_B = max(0.0, (self.esc_B - 1000) / 800.0 * 2.0)
            # courant_moteur_C = max(0.0, (self.esc_C - 1000) / 800.0 * 2.0)
            # courant_moteur_D = max(0.0, (self.esc_D - 1000) / 800.0 * 2.0)
            
            # courant_total = courant_moteur_A + courant_moteur_B + courant_moteur_C + courant_moteur_D

            # consommation_pas = courant_total * (self.delta_t / 3600.0)
            # self.capacite_restante_Ah = max(0.0, self.capacite_restante_Ah - consommation_pas)
            # self.soc_batterie = (self.capacite_restante_Ah / self.capacite_batterie_Ah) * 100.0

            # tension_repos = self.tension_min + (self.tension_max - self.tension_min) * (self.soc_batterie / 100.0)
            # self.tension_actuelle = tension_repos - (self.resistance_interne * courant_total)

            # if self.tension_actuelle <= self.tension_min:
            #     self.tension_actuelle = self.tension_min
            #     self.batterie_critique = True

            # if courant_total > self.courant_max_batterie:
            #     facteur_limitation = self.courant_max_batterie / courant_total
            #     self.f_A *= facteur_limitation
            #     self.f_B *= facteur_limitation
            #     self.f_C *= facteur_limitation
            #     self.f_D *= facteur_limitation

        else:
            self.f_A = self.f_B = self.f_C = self.f_D = 0.0
            self.esc_A = self.esc_B = self.esc_C = self.esc_D = 1000.0
            # courant_total = 0.0
            # self.tension_actuelle = self.tension_min + (self.tension_max - self.tension_min) * (self.soc_batterie / 100.0)

        vitesse_lin_norme = np.linalg.norm(vit)
        F_frot = -self.k_frot * vitesse_lin_norme * vit
        self.poussee_totale = self.f_A + self.f_B + self.f_C + self.f_D
        R = fct.get_rotation_matrix(self.angle[0], self.angle[1], self.angle[2])

        Forces = np.array([
            np.dot(R, [0, 0, self.f_A]), np.dot(R, [0, 0, self.f_B]),
            np.dot(R, [0, 0, self.f_C]), np.dot(R, [0, 0, self.f_D]), self.Poids_global, F_frot
        ])
        
        # Application des moments en tenant compte du barycentre décalé
        # Moments = fct.moments_moteur(
        #     np.array([0, 0, self.f_A]), np.array([0, 0, self.f_B]),
        #     np.array([0, 0, self.f_C]), np.array([0, 0, self.f_D]), 
        #     self.L, self.l, 
        #     b_x=self.barycentre[0], b_y=self.barycentre[1], b_z=self.barycentre[2],
        #     h_helice=self.hauteur_helice
        # )

        Moments = fct.moments_moteur(
            np.array([0, 0, self.f_A]), np.array([0, 0, self.f_B]),
            np.array([0, 0, self.f_C]), np.array([0, 0, self.f_D]), 
            self.L, self.l, 
            b_x=self.barycentre[0], b_y=self.barycentre[1], b_z=self.barycentre[2],
            h_helice=self.hauteur_helice
        )

        pos_ancienne, self.pos_actuel = np.copy(self.pos_actuel), np.copy(self.pos)
        angle_ancien, self.angle_actuel = np.copy(self.angle_actuel), np.copy(self.angle)

        self.pos = dyn.position(self.pos_actuel, pos_ancienne, Forces, self.Masse, Delta_t=self.delta_t)
        self.angle = dyn.angle(self.angle_actuel, angle_ancien, Moments, self.Mat_Inertie, Delta_t=self.delta_t)

        if self.pos[2] <= 0:
            self.pos[2] = 0.0
            if self.poussee_totale < (self.Masse * self.g):
                self.pos[0], self.pos[1] = self.pos_actuel[0], self.pos_actuel[1]
                self.angle = np.copy(self.angle_actuel)

        self.vitesse = (self.pos - self.pos_actuel) / self.delta_t
        self.norme_vitesse = float(np.linalg.norm(self.vitesse))

        vitesse_angulaire_rad = (self.angle - self.angle_actuel) / self.delta_t
        self.vitesse_angulaire_deg = np.degrees(vitesse_angulaire_rad)
        self.alerte_vitesse_angulaire = bool(np.any(np.abs(self.vitesse_angulaire_deg[:2]) > SEUIL_VIT_ANGULAIRE_MAX))

        self.hist_roll.append(float(np.degrees(self.angle[0])))
        self.hist_pitch.append(float(np.degrees(self.angle[1])))
        if len(self.hist_roll) > 100:
            self.hist_roll.pop(0)
            self.hist_pitch.pop(0)

        self.t += self.delta_t


def commande(temps):

    if temps < 2:
        return 0, 0, 0

    elif temps < 4:
        return 1, 0, 0

    elif temps < 5:
        return 0, 0, 0

    elif temps < 6:
        return -1, 0, 0

    else:
        return 0, 0, 0

def angle_perturb(temps, angle):
    if temps < 2 :
        return angle
    elif temps < 2.3:
        return 10.0 * np.pi/180

    else:
        return angle

def test_stab(liste_bool):
    imax = 0

    for i in range (1, len(liste_bool)):
        if liste_bool[i] and not(liste_bool[i-1]):
            imax = i

    return imax


if __name__ == "__main__":
    # 1. Initialisation de la simulation
    simu = DroneSimulation()
    simu.toggle_motors()  # Démarre le drone (sinon les moteurs restent à 0)

    simu.pid_gains["roll"]["kp"] = 15.0
    simu.pid_gains["roll"]["ki"] = 0.5
    simu.pid_gains["roll"]["kd"] = 1.2

    simu.pid_gains["pitch"]["kp"] = 0.0 # 80.0 
    simu.pid_gains["pitch"]["ki"] = 0.0 # 0.8
    simu.pid_gains["pitch"]["kd"] = 0.0 #19.0

    simu.angle_max = 15.0

    angle = "roll"

    fichier = open(f"Results_stabilization/simulation_{angle}_{simu.angle_max}deg.txt", "w")

    # 2. Paramètres de la boucle
    duree_simulation = 12.0 # Durée en secondes
    nb_pas = int(duree_simulation / simu.delta_t)

    t = 0

    # Commande des joysticks : (roll, pitch, throttle)
    stick_gx = 0.0  # Léger ordre de roulis
    stick_gy = 0.0
    stick_dy = 0.0

    liste_angle = []
    list_escA = []
    list_escB = []
    list_escC = []
    list_escD = []

    list_tension = []

    # 3. Boule de simulation
    for _ in range(nb_pas):
        stick_gx, stick_gy, stick_dy = commande(t) #0 ,0, 0

        simu.step(stick_gx=stick_gx, stick_gy=stick_gy, stick_dy=stick_dy)
        simu.pos[2] = 5

        # simu.angle = np.array([angle_perturb(t, simu.angle[0]), 0, 0])

        # print(simu.angle)

        t += simu.delta_t

        yaw_deg = np.degrees(simu.angle[2])
        pitch_deg = np.degrees(simu.angle[1])
        roll_deg = np.degrees(simu.angle[0])

        liste_angle.append(roll_deg)
        list_escA.append(simu.esc_A)
        list_escB.append(simu.esc_B)
        list_escC.append(simu.esc_C)
        list_escD.append(simu.esc_D)
        

        list_tension.append(simu.tension_actuelle)

        fichier.write(f" {roll_deg:.2f}, {pitch_deg:.2f},  {yaw_deg:.2f} \n")


    liste_temps = np.array([i for i in range (len(liste_angle))]) * simu.delta_t

    mask_temps = liste_temps > 4
    liste_angle = np.array(liste_angle)
    # print(liste_angle[mask_temps])
    mask_roulis_stab = abs(liste_angle[mask_temps]) < 0.1
    mask_roulis_stab = np.concatenate([[False for _ in range(len(liste_temps)-len(mask_roulis_stab))], mask_roulis_stab])

    mask_roulis_cmd = abs(abs(liste_angle) - simu.angle_max) < 0.1

    cmd_angle = [1 * commande(liste_temps[i])[0]* simu.angle_max for i in range (len(liste_temps))]

    fichier.write("############################ \n")
    fichier.write("Stabilization \n")
    fichier.write("############################ \n")
    fichier.write(f"cmd : {liste_temps[test_stab(mask_roulis_cmd)]}, back to 0deg : {liste_temps[test_stab(mask_roulis_stab)]}\n")
    fichier.write(f"Time stabilization \n")
    fichier.write(f"cmd : {(liste_temps[test_stab(mask_roulis_cmd)] - 2.0):.3f}, back to 0deg : {(liste_temps[test_stab(mask_roulis_stab)] - 4.0):.3f} \n")

    fichier.close()

   # Définition d'un nombre fixe de points pour l'affichage (ex: 1000 points)
    nb_points_affichage = 6000
    temps_plot = np.linspace(liste_temps[0], liste_temps[-1], nb_points_affichage)

    # Interpolation linéaire des signaux sur la nouvelle grille de temps
    angle_plot = np.interp(temps_plot, liste_temps, liste_angle)
    cmd_plot = np.interp(temps_plot, liste_temps, cmd_angle)
    list_escA_plot = np.interp(temps_plot, liste_temps, list_escA)
    list_escB_plot = np.interp(temps_plot, liste_temps, list_escB)
    list_escC_plot = np.interp(temps_plot, liste_temps, list_escC)
    list_escD_plot = np.interp(temps_plot, liste_temps, list_escD)
    list_tension_plot = np.interp(temps_plot, liste_temps, list_tension)


    fig, axes = plt.subplots(3, 1, sharex=True)

    axes[0].plot(
    temps_plot,
    angle_plot,
    label=f"{angle} angle",
    color='blue',
    linewidth=1.5,
    )
    axes[0].plot(
        temps_plot,
        cmd_plot,
        label=f"{angle} angle",
        color='black',
        linestyle = "--",
        linewidth=1.5,
        )
    axes[0].set_ylabel(f"{angle} angle (deg)")
    axes[0].set_title(f"{angle} angle (deg) Simulation")
    axes[0].legend(loc='upper right')
    axes[0].grid(True)

    axes[1].plot(
        temps_plot,
        list_escA_plot,
        label='esc A',
        linewidth=1.5,
    )    

    axes[1].plot(
            temps_plot,
            list_escB_plot,
            label='esc B',
            linewidth=1.5,
        )
    axes[1].plot(
                temps_plot,
                list_escC_plot,
                label='esc C',
                linewidth=1.5,
            )
    axes[1].plot(
                temps_plot,
                list_escD_plot,
                label='esc D',
                linewidth=1.5,
            )

    axes[1].set_ylabel(f"esc command")
    axes[1].set_title(f"esc")
    axes[1].legend(loc='upper right')
    axes[1].grid(True)

    axes[2].plot(
                temps_plot,
                (np.array(list_escB_plot) - 1150)/1150 * 100,
                label='pourcentage esc',
                linewidth=1.5,
            )
    
    axes[2].set_ylabel(f"%")
    axes[2].set_title(f"% esc")
    axes[2].legend(loc='upper right')
    axes[2].grid(True)

    
    # # Les vlines utilisent les valeurs temporelles exactes
    # plt.vlines([liste_temps[test_stab(mask_roulis_stab)]], -50, 50, colors="red")
    # plt.vlines([liste_temps[test_stab(mask_roulis_cmd)]], -50, 50, colors="red")
    
    # plt.tight_layout()
    plt.show()

    # 4. Affichage des résultats après simulation
    print(f"Temps écoulé : {simu.t:.2f} s")
    print(f"Position (X, Y, Z) : {-simu.pos[1]:.2f}, {-simu.pos[0]:.2f}, {simu.pos[2]:.2f}")
    
    # Angles en degrés (Yaw, Pitch, Roll)
    yaw_deg = np.degrees(simu.angle[2])
    pitch_deg = np.degrees(simu.angle[1])
    roll_deg = np.degrees(simu.angle[0])
    print(f"Angles (Yaw, Pitch, Roll) en deg : {yaw_deg:.2f}°, {pitch_deg:.2f}°, {roll_deg:.2f}°")