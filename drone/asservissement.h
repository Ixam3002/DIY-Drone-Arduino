#include <Arduino.h>

// Structure pour retourner les corrections des 4 moteurs
struct CommandesMoteurs {
  int16_t f_corr_A;
  int16_t f_corr_B;
  int16_t f_corr_C;
  int16_t f_corr_D;
};

// Variables globales d'état du PID
float erreur_integrale_roll = 0.0;
float erreur_integrale_pitch = 0.0;
float erreur_integrale_yaw = 0.0;
float erreur_integrale_alt = 0.0;

float erreur_prec_roll = 0.0;
float erreur_prec_pitch = 0.0;
float erreur_prec_yaw = 0.0;
float erreur_prec_alt = 0.0;

CommandesMoteurs calcul_correction_PID(
  const float angle_actuel[3],
  float vitesse_Z,
  bool altitude_corr = true,
  const float angle_cible[3] = (const float[]){0.0, 0.0, 0.0},
  float Kp_roll = 0.2, float Kp_pitch = 0.2, float Kp_yaw = 0.1, float Kp_alt = 0.1,
  float Ki_roll = 0.01, float Ki_pitch = 0.01, float Ki_yaw = 0.005, float Ki_alt = 0.005,
  float Kd_roll = 0.05, float Kd_pitch = 0.05, float Kd_yaw = 0.02, float Kd_alt = 0.02,
  float delta_t = 0.05,
  bool reset_pid = false
) {
  // 1. Calcul des erreurs
  float erreur_roll  = angle_cible[0] - angle_actuel[0];
  float erreur_pitch = angle_cible[1] - angle_actuel[1];
  float erreur_yaw   = angle_cible[2] - angle_actuel[2];
  float erreur_alt   = 0.0 - vitesse_Z;

  // Réinitialisation des états (intégrale et dérivée)
  if (reset_pid) {
    erreur_integrale_roll  = 0.0;
    erreur_integrale_pitch = 0.0;
    erreur_integrale_yaw   = 0.0;
    erreur_integrale_alt   = 0.0;

    erreur_prec_roll  = erreur_roll;
    erreur_prec_pitch = erreur_pitch;
    erreur_prec_yaw   = erreur_yaw;
    erreur_prec_alt   = erreur_alt;
  }

  // 2. Terme Intégral (accumulation)
  erreur_integrale_roll  += erreur_roll * delta_t;
  erreur_integrale_pitch += erreur_pitch * delta_t;
  erreur_integrale_yaw   += erreur_yaw * delta_t;
  erreur_integrale_alt   += erreur_alt * delta_t;

  float limite_integrale = 1.0;
  erreur_integrale_roll  = constrain(erreur_integrale_roll, -limite_integrale, limite_integrale);
  erreur_integrale_pitch = constrain(erreur_integrale_pitch, -limite_integrale, limite_integrale);
  erreur_integrale_yaw   = constrain(erreur_integrale_yaw, -limite_integrale, limite_integrale);
  erreur_integrale_alt   = constrain(erreur_integrale_alt, -limite_integrale, limite_integrale);

  // 3. Terme Dérivatif
  float d_erreur_roll  = (erreur_roll - erreur_prec_roll) / delta_t;
  float d_erreur_pitch = (erreur_pitch - erreur_prec_pitch) / delta_t;
  float d_erreur_yaw   = (erreur_yaw - erreur_prec_yaw) / delta_t;
  float d_erreur_alt   = (erreur_alt - erreur_prec_alt) / delta_t;

  // Sauvegarde des erreurs courantes pour le pas suivant
  erreur_prec_roll  = erreur_roll;
  erreur_prec_pitch = erreur_pitch;
  erreur_prec_yaw   = erreur_yaw;
  erreur_prec_alt   = erreur_alt;

  // 4. Commandes de correction PID
  float corr_roll  = (Kp_roll * erreur_roll)   + (Ki_roll * erreur_integrale_roll)   + (Kd_roll * d_erreur_roll);
  float corr_pitch = (Kp_pitch * erreur_pitch) + (Ki_pitch * erreur_integrale_pitch) + (Kd_pitch * d_erreur_pitch);
  float corr_yaw   = (Kp_yaw * erreur_yaw)     + (Ki_yaw * erreur_integrale_yaw)     + (Kd_yaw * d_erreur_yaw);
  float corr_alt   = (Kp_alt * erreur_alt)     + (Ki_alt * erreur_integrale_alt)     + (Kd_alt * d_erreur_alt);

  // 5. Répartition sur les 4 moteurs (A, B, C, D)
  CommandesMoteurs cmd;

  if (altitude_corr) {
    cmd.f_corr_A = -corr_roll - corr_pitch - corr_yaw + corr_alt;
    cmd.f_corr_B = +corr_roll - corr_pitch + corr_yaw + corr_alt;
    cmd.f_corr_C = -corr_roll + corr_pitch + corr_yaw + corr_alt;
    cmd.f_corr_D = +corr_roll + corr_pitch - corr_yaw + corr_alt;
  } else {
    cmd.f_corr_A = -corr_roll - corr_pitch - corr_yaw;
    cmd.f_corr_B = +corr_roll - corr_pitch + corr_yaw;
    cmd.f_corr_C = -corr_roll + corr_pitch + corr_yaw;
    cmd.f_corr_D = +corr_roll + corr_pitch - corr_yaw;
  }

  return cmd;
}