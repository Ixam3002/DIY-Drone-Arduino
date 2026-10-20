#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h> // <--- 1. Inclure la bibliothèque EEPROM

#include "asservissement.h"
#include "fonctions.h"
#include "MPU6050.h"
#include "conv_joystick2angle.h"

const int PIN_BATTERY = A2;


RF24 radio(19, 18);
const byte adresse[6] = "00001";

//Interruption Radio
volatile bool NewMessage = false;

//Definition des esc
Servo escA, escB, escC, escD;
const int ESCA_PIN = 9, ESCB_PIN = 5, ESCC_PIN = 10, ESCD_PIN = 6;

const float ANGLE_MAX = 30.0; //deg

// Structure pour stocker proprement les PID en EEPROM
struct PIDConfig {
  float kp_pitch, ki_pitch, kd_pitch;
  float kp_roll, ki_roll, kd_roll;
};

float kp_pitch = 0.0, ki_pitch = 0.0, kd_pitch = 0.0;
float kp_roll  = 0.75,  ki_roll  = 0.005,  kd_roll  = 1.9;
float kp_alt   = 4.98980340e+01,  ki_alt   = 8.24779880e-03,  kd_alt   = 1.34603797e+00;

bool Stab_Alt = false;
bool ARMED = false;
int16_t Throttle = 1150;

float stick_lx = 0; float stick_ly = 0; float stick_rx = 0; float stick_ry = 0;
bool button_RB = false; bool button_A = false; bool button_B = false;
bool button_RB_prec = false;

int16_t value_escA = 0; int16_t value_escB = 0 ; int16_t value_escC = 0 ; int16_t value_escD = 0;
float target_angle[3]; float angles[3]; float accel[3];

struct __attribute__((packed)) TelemetrieCompact { //32 octets MAX
  int16_t roll; // 2 octets
  int16_t pitch; // 2 octets
  int16_t rollTarget; // 2 octets
  int16_t pitchTarget; // 2 octets
  int16_t Throttle; // 2 octets
  int16_t escA; // 2 octets
  int16_t escB; // 2 octets
  int16_t escC; // 2 octets
  int16_t escD; // 2 octets
  int16_t battery; // 2 octet
  float tps_boucle; // 4 octets
  uint8_t armed; // 1 octet
};

TelemetrieCompact telMsg;

CommandesMoteurs motors;
MpuData mpu;

// Variable globale pour conserver l'état filtré
float battery_filtered = 0.0;

struct __attribute__((packed)) Packet {
  uint8_t header; // 1 octet
  int16_t lx, ly, rx, ry; // 8 octets
  uint8_t rb; // 1 octet
  uint8_t button_A; //1 octet
  uint8_t button_B; // 1 octet
  uint16_t pitch_kp, pitch_ki, pitch_kd; // 6 octets
  uint16_t roll_kp, roll_ki, roll_kd; // 6 octets
};


void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_BATTERY, INPUT);

  Serial.println("radio");

  // --- 2. Chargement des PID depuis l'EEPROM au démarrage ---
  PIDConfig savedPID;
  EEPROM.get(0, savedPID);
  
  // Vérification basique si l'EEPROM n'est pas vierge (NaN)
  if (!isnan(savedPID.kp_roll) && savedPID.kp_roll >= 0 && savedPID.kp_roll <= 100) {
    kp_pitch = savedPID.kp_pitch;
    ki_pitch = savedPID.ki_pitch;
    kd_pitch = savedPID.kd_pitch;
    kp_roll  = savedPID.kp_roll;
    ki_roll  = savedPID.ki_roll;
    kd_roll  = savedPID.kd_roll;
    Serial.print("Gains PID charges depuis l'EEPROM !");
  } else {
    Serial.println("Premier demarrage ou EEPROM vide, utilisation des valeurs par defaut.");
  }

  if (!radio.begin()) {
    Serial.println("NRF24L01 non detecte !");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  
  // Configuration des options ACK
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  
  radio.openReadingPipe(0, adresse);
  radio.startListening();

  // Pré-charger une première fois l'état "armed" dans l'ACK
  telMsg.roll = 0;
  telMsg.pitch = 0;
  telMsg.rollTarget = 0;
  telMsg.pitchTarget = 0;
  telMsg.Throttle = Throttle;
  telMsg.escA = 0;
  telMsg.escB = 0;
  telMsg.escC = 0;
  telMsg.escD = 0;
  telMsg.tps_boucle = 0;
  telMsg.armed = ARMED ? 1 : 0;
  radio.writeAckPayload(0, &telMsg, sizeof(telMsg));

  Serial.println("Recepteur pret");

  // Attache des esc au moteur
  escA.attach(ESCA_PIN, 1000, 2000);
  escB.attach(ESCB_PIN, 1000, 2000);
  escC.attach(ESCC_PIN, 1000, 2000);
  escD.attach(ESCD_PIN, 1000, 2000);

  escA.writeMicroseconds(1000);
  escB.writeMicroseconds(1000);
  escC.writeMicroseconds(1000);
  escD.writeMicroseconds(1000);

  initMPU();
  delay(500);

  Serial.println("FIN");
}

void loop() {
  uint32_t currentMicros = micros();

  if (radio.available()) {
    Packet p;
    radio.read(&p, sizeof(p));
    
    stick_lx = (float)p.lx/100.0f;
    stick_ly = (float)p.ly/100.0f;
    stick_rx = (float)p.rx/100.0f;
    stick_ry = (float)p.ry/100.0f;
    button_RB = p.rb;
    button_A = p.button_A;
    button_B = p.button_B;

    // Mise à jour de l'état ARMED avec le bouton RB
    if (button_RB && !button_RB_prec) {
      ARMED = !ARMED; 
    }
    button_RB_prec = button_RB;

    if (button_A && button_B && !ARMED){
      // Correction des assignations (ki et kd utilisaient kp par erreur avant)
      kp_pitch = (float)p.pitch_kp / 100.0f;
      ki_pitch = (float)p.pitch_ki / 100.0f;
      kd_pitch = (float)p.pitch_kd / 100.0f;
      kp_roll  = (float)p.roll_kp  / 100.0f;
      ki_roll  = (float)p.roll_ki  / 100.0f;
      kd_roll  = (float)p.roll_kd  / 100.0f;

      // --- 3. Sauvegarde automatique dans l'EEPROM ---
      PIDConfig newPID = {kp_pitch, ki_pitch, kd_pitch, kp_roll, ki_roll, kd_roll};
      EEPROM.put(0, newPID);

      Serial.println("Changement de configuration PID et sauvegarde EEPROM effectues !");
    }

  }

  CommandJoystick cmd = joystick2angle(stick_lx, stick_ly, stick_rx, stick_ry, ANGLE_MAX);


  MPU_Angles_ACC(mpu);
  angles[0] = mpu.roll;
  angles[1] = mpu.pitch;
  angles[2] = mpu.yaw;

  Throttle += cmd.throttle;
  Throttle = throttle_boundary(Throttle);

  target_angle[0] = cmd.roll;
  target_angle[1] = cmd.pitch;
  target_angle[2] = cmd.yaw;

  

  if (ARMED) {

    motors = calcul_correction_PID(angles, 0.0, Stab_Alt, target_angle,
                                   kp_roll, kp_pitch, 0.1, kp_alt,
                                   ki_roll, ki_pitch, 0.005, ki_alt,
                                   kd_roll, kd_pitch, 0.02, kd_alt);

    value_escA = esc_boundary(Throttle + motors.f_corr_A);
    value_escB = correction_motorB(Throttle + motors.f_corr_B);
    value_escC = esc_boundary(Throttle + motors.f_corr_C);
    value_escD = esc_boundary(Throttle + motors.f_corr_D);

    escA.writeMicroseconds(value_escA);
    escB.writeMicroseconds(value_escB);
    escC.writeMicroseconds(value_escC);
    escD.writeMicroseconds(value_escD);
  } else {
    escA.writeMicroseconds(1000);
    escB.writeMicroseconds(1000);
    escC.writeMicroseconds(1000);
    escD.writeMicroseconds(1000);
  }

  uint32_t endMicros = micros();

  // IMPORTANT : Recharger immédiatement le payload ACK pour le prochain échange
  telMsg.roll = angles[0] * 100;
  telMsg.pitch = angles[1] * 100;
  telMsg.rollTarget = target_angle[0] * 100;
  telMsg.pitchTarget = target_angle[1] * 100;
  telMsg.Throttle = Throttle;
  telMsg.escA = value_escA;
  telMsg.escB = value_escB;
  telMsg.escC = value_escC;
  telMsg.escD = value_escD;


  // low pass filter for battery
  int raw_battery = ((analogRead(PIN_BATTERY) * 5.0f) / 1023.0f)*100;
  // Filtre passe-bas numérique (EMA) :
  // alpha = 0.05 détermine la force du lissage (entre 0.01 et 1.0)
  // - Plus alpha est PETIT (ex: 0.01), plus c'est LISSÉ mais réagit lentement.
  // - Plus alpha est GRAND (ex: 0.2), plus c'est RÉACTIF mais laisse passer du bruit.
  float alpha = 0.05;
  battery_filtered = (alpha * raw_battery) + ((1.0 - alpha) * battery_filtered);

  telMsg.battery = (int16_t)battery_filtered;
  telMsg.tps_boucle = endMicros - currentMicros;
  telMsg.armed = ARMED ? 1 : 0;
  radio.writeAckPayload(0, &telMsg, sizeof(telMsg));

}