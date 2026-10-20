#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(19, 18);   // CE, CSN
const byte adresse[6] = "00001";

// ---------------- Commandes envoyees au drone ----------------
struct __attribute__((packed)) Packet {
  uint8_t header;
  float lx, ly, rx, ry;
  uint8_t rb;
  uint8_t checksum;
};

// ---------------- Structure de telemetrie reçue ----------------
struct __attribute__((packed)) TelemetrieCompact {
  float roll;
  float pitch;
  float throttle;
  uint8_t armed;
};

Packet pkt;
TelemetrieCompact telData;

uint8_t buf[sizeof(Packet)];
uint8_t idx = 0;
bool fresh = false;

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);    // USB -> moniteur serie
  Serial1.begin(115200);   // UART depuis le XIAO

  unsigned long t = millis();
  while (!Serial && millis() - t < 2000);

  if (!radio.begin()) {
    if (Serial) Serial.println("NRF24L01 non detecte !");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  
  // Configuration pour emission et reception sur la même adresse
  radio.openWritingPipe(adresse);
  radio.openReadingPipe(1, adresse);
  radio.enableAckPayload(); // Active l'envoi de données dans l'accusé de réception
  
  radio.stopListening(); // Commence en mode émission

  if (Serial) Serial.println("Emetteur pret avec telemetrie");
}

void loop() {
  // ---- Lecture UART depuis le XIAO ----
  while (Serial1.available()) {
    uint8_t c = Serial1.read();

    if (idx == 0 && c != 0xAA) continue;

    buf[idx++] = c;

    if (idx == sizeof(Packet)) {
      idx = 0;

      uint8_t cs = 0;
      for (uint8_t i = 1; i < sizeof(Packet) - 1; i++) cs ^= buf[i];

      if (cs == buf[sizeof(Packet) - 1]) {
        memcpy(&pkt, buf, sizeof(Packet));
        fresh = true;
      }
    }
  }

  // ---- Envoi radio avec écoute de la télémétrie en retour (ACK Payload) ----
  radio.write(&pkt, sizeof(pkt));
  
  // Si le drone a inclus des données de télémétrie dans l'accusé de réception (ACK)
  if (radio.isAckPayloadAvailable()) {
    radio.read(&telData, sizeof(telData));
  }

  if (fresh) {
    fresh = false;
  }

  // ---- Affichage de la télémétrie et des commandes sur le moniteur série ----
  if (Serial && millis() - lastPrint >= 100) {
    lastPrint = millis();

    Serial.print("LX:");   Serial.print(pkt.lx, 2);
    Serial.print(" | Roll:"); Serial.print(telData.roll, 1);
    Serial.print(" | Pitch:"); Serial.print(telData.pitch, 1);
    Serial.print(" | Thr:"); Serial.print(telData.throttle, 1);
    Serial.print(" | Armed:"); Serial.println(telData.armed);
  }
}