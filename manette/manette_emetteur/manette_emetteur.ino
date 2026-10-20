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

// ---------------- Telemetrie recue du drone (12 octets) ----------------
struct __attribute__((packed)) Telemetry {
  int16_t  pitch;      // degres x100
  int16_t  roll;       // degres x100
  uint16_t motor[4];
};

Packet pkt;
Telemetry tele;

uint8_t buf[sizeof(Packet)];
uint8_t idx = 0;
bool fresh = false;

unsigned long lastTele = 0;    // derniere telemetrie recue
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);    // USB -> moniteur serie (facultatif)
  Serial1.begin(115200);   // UART depuis le XIAO (broche 0 = RX)

  // Attend le moniteur 2 s maximum, puis continue sans
  unsigned long t = millis();
  while (!Serial && millis() - t < 2000);

  if (!radio.begin()) {
    if (Serial) Serial.println("NRF24L01 non detecte !");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.enableDynamicPayloads();   // meme reglage que le drone
  radio.enableAckPayload();
  radio.setRetries(4, 5);          // 1250 us : assez long pour un ACK avec donnees a 250 kbps
  radio.openWritingPipe(adresse);
  radio.stopListening();

  if (Serial) Serial.println("Emetteur pret");
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

  // ---- Envoi radio + lecture de la telemetrie dans l'accuse de reception ----
  if (radio.write(&pkt, sizeof(pkt))) {
    if (radio.available()) {
      radio.read(&tele, sizeof(tele));
      lastTele = millis();
    }
  }

  if (fresh) {
    fresh = false;
  }

  // ---- Affichage uniquement si le moniteur est ouvert ----
  if (Serial && millis() - lastPrint >= 100) {
    lastPrint = millis();

    Serial.print("LX:");   Serial.print(pkt.lx, 2);
    Serial.print("  LY:"); Serial.print(pkt.ly, 2);
    Serial.print("  RX:"); Serial.print(pkt.rx, 2);
    Serial.print("  RY:"); Serial.print(pkt.ry, 2);
    Serial.print("  RB:"); Serial.print(pkt.rb);

    if (millis() - lastTele < 500) {
      Serial.print("  | Pitch:"); Serial.print(tele.pitch / 100.0f, 1);
      Serial.print("  Roll:");    Serial.print(tele.roll / 100.0f, 1);
      Serial.print("  M:");
      for (uint8_t i = 0; i < 4; i++) {
        Serial.print(tele.motor[i]);
        Serial.print(i < 3 ? '/' : ' ');
      }
      Serial.println();
    } else {
      Serial.println("  | Pas de telemetrie");
    }
  }
}
