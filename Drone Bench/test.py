"""
simple_serial_read.py
---------------------
Programme minimaliste de lecture du port série avec Python.
"""

import serial
import time

# Remplacez "COM3" par votre port série (ex: "COM4" sous Windows ou "/dev/ttyUSB0" sous Linux)
PORT = "COM11"
BAUDRATE = 115200

def read_serial_port():
    print(f"Connexion au port {PORT} à {BAUDRATE} bauds...")
    
    try:
        # Ouverture de la liaison série
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(2) # Laisse le temps à l'Arduino de redémarrer
        print("Connecté ! En attente de données (Ctrl+C pour quitter)...\n")
        
        while True:
            if ser.in_waiting > 0:
                # Lecture d'une ligne, décodage en texte UTF-8 et suppression des espaces/retours à la ligne superflus
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"Reçu : {line}")
                    
    except serial.SerialException as e:
        print(f"Erreur de port série : {e}")
    except KeyboardInterrupt:
        print("\nArrêt du programme par l'utilisateur.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Port série fermé.")

if __name__ == "__main__":
    read_serial_port()