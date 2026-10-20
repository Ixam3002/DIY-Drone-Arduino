"""
telemetry_serial_2.py
---------------------
Moniteur de télémétrie ultra-rapide haute performance (PyQt5 + PyQtGraph).
- Lecture binaire optimisée via le module struct
- Historique graphique étendu à 600 points
- Sauvegarde CSV complète (angles + moteurs ESC A, B, C, D)
- Contrôle PID et liaison série multi-threads
- Affichage du temps de boucle
"""

import sys
import serial
import threading
import csv
import datetime
import struct
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

class SerialTelemetryApp(QtWidgets.QMainWindow):
    def __init__(self, port="COM9", baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.is_connected = False
        
        # Enregistrement CSV
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        
        # Données de télémétrie
        self.drone_demarre = False
        self.vitesse = (0.0, 0.0, 0.0)
        self.norme_vitesse = 0.0
        self.vitesse_angulaire_deg = (0.0, 0.0)
        self.lx = self.ly = self.rx = self.ry = 0.0
        self.esc_A = self.esc_B = self.esc_C = self.esc_D = 1000.0
        self.throttle = 1000.0
        self.tps_boucle = 0.0  # Variable pour stocker le temps de boucle
        self.battery_voltage = 0.0
        
        # Tampons pour les graphiques étendus à 600 points
        self.max_points = 600
        self.time_data = list(range(self.max_points))
        self.hist_roll = [0.0] * self.max_points
        self.hist_pitch = [0.0] * self.max_points
        self.hist_target_roll = [0.0] * self.max_points
        self.hist_target_pitch = [0.0] * self.max_points
        
        self.pid_gains = {
            "roll": {"kp": 1.6, "ki": 1.2, "kd": 2.2},
            "pitch": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
            "alt": {"kp": 49.89, "ki": 0.0082, "kd": 1.346}
        }

        self.init_ui()
        self.start_serial_thread()

    def init_ui(self):
        self.setWindowTitle("Moniteur Télémétrie Binaire (PyQtGraph)")
        self.resize(720, 980)
        self.setStyleSheet("background-color: #12161c; color: #ffffff; font-family: Consolas;")

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)

        # 1. En-tête Statut & Connexion
        top_layout = QtWidgets.QHBoxLayout()
        self.lbl_status = QtWidgets.QLabel("MOTORS: OFFLINE")
        self.lbl_status.setStyleSheet("font-size: 14pt; font-weight: bold; color: #ff4d4d;")
        top_layout.addWidget(self.lbl_status)

        top_layout.addStretch()

        self.btn_connect = QtWidgets.QPushButton("Connexion")
        self.btn_connect.setStyleSheet("background-color: #00ff66; color: #12161c; font-weight: bold; padding: 6px;")
        self.btn_connect.clicked.connect(self.toggle_connection)
        top_layout.addWidget(self.btn_connect)
        main_layout.addLayout(top_layout)

        # 2. Vitesses & Temps de boucle
        group_speeds = QtWidgets.QGroupBox(" Système & Vitesses ")
        group_speeds.setStyleSheet("QGroupBox { color: #00d2ff; font-weight: bold; border: 1px solid #1a212b; margin-top: 5px; } QGroupBox::title { subcontrol-position: top left; padding: 0 3px; }")
        speed_layout = QtWidgets.QVBoxLayout(group_speeds)
        
        self.lbl_vitesse = QtWidgets.QLabel("Vx: +0.00 m/s | Vy: +0.00 m/s | Vz: +0.00 m/s | ||V||: 0.00 m/s")
        self.lbl_angulaire = QtWidgets.QLabel("Wx (Roll): +0.0°/s\nWy (Pitch): +0.0°/s")
        
        # Ajout du QLabel pour le temps de boucle
        self.lbl_tps_boucle = QtWidgets.QLabel("Temps de boucle: 0.00 ms")
        self.lbl_tps_boucle.setStyleSheet("color: #00ff66; font-weight: bold; font-size: 10pt;")

        #ajout du QLabel pour la tension de la batterie
        self.lbl_battery = QtWidgets.QLabel("Batterie: 0.00 V")
        self.lbl_battery.setStyleSheet("color: #ff55ff; font-weight: bold; font-size: 10pt;")

        speed_layout.addWidget(self.lbl_vitesse)
        speed_layout.addWidget(self.lbl_angulaire)
        speed_layout.addWidget(self.lbl_tps_boucle)
        speed_layout.addWidget(self.lbl_battery)
        main_layout.addWidget(group_speeds)

        # 3. Joysticks
        group_joy = QtWidgets.QGroupBox(" Commandes Manette (Joysticks) ")
        group_joy.setStyleSheet("QGroupBox { color: #00d2ff; font-weight: bold; border: 1px solid #1a212b; margin-top: 5px; } QGroupBox::title { subcontrol-position: top left; padding: 0 3px; }")
        joy_layout = QtWidgets.QVBoxLayout(group_joy)
        self.lbl_joystick = QtWidgets.QLabel("Joystick Gauche  -> LX: +0.00 | LY: +0.00\nJoystick Droit   -> RX: +0.00 | RY: +0.00")
        self.lbl_joystick.setStyleSheet("color: #ffcc00;")
        joy_layout.addWidget(self.lbl_joystick)
        main_layout.addWidget(group_joy)

        # 4. Moteurs & Throttle (Barres de progression)
        group_motors = QtWidgets.QGroupBox(" Commandes Moteurs & Throttle ")
        group_motors.setStyleSheet("QGroupBox { color: #00d2ff; font-weight: bold; border: 1px solid #1a212b; margin-top: 5px; } QGroupBox::title { subcontrol-position: top left; padding: 0 3px; }")
        motors_layout = QtWidgets.QVBoxLayout(group_motors)
        
        h_layout_bars = QtWidgets.QHBoxLayout()

        bars_layout = QtWidgets.QHBoxLayout()
        self.motor_bars = {}
        for m_name in ["A", "B", "C", "D"]:
            v_box = QtWidgets.QVBoxLayout()
            lbl = QtWidgets.QLabel(f"Moteur {m_name}")
            lbl.setStyleSheet("color: #ffcc00; font-size: 9pt;")
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            v_box.addWidget(lbl)

            bar = QtWidgets.QProgressBar()
            bar.setOrientation(QtCore.Qt.Vertical)
            bar.setRange(1000, 2000)
            bar.setValue(1000)
            bar.setFixedSize(30, 90)
            bar.setStyleSheet("QProgressBar { background: #1a212b; border: 1px solid #2a3442; } QProgressBar::chunk { background: #00d2ff; }")
            v_box.addWidget(bar, 0, QtCore.Qt.AlignCenter)

            val_lbl = QtWidgets.QLabel("1000 ESC")
            val_lbl.setStyleSheet("font-size: 8pt;")
            val_lbl.setAlignment(QtCore.Qt.AlignCenter)
            v_box.addWidget(val_lbl)

            bars_layout.addLayout(v_box)
            self.motor_bars[m_name] = (bar, val_lbl)
        
        h_layout_bars.addLayout(bars_layout)

        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.VLine)
        line.setFrameShadow(QtWidgets.QFrame.Sunken)
        line.setStyleSheet("color: #2a3442;")
        h_layout_bars.addWidget(line)

        throttle_vbox = QtWidgets.QVBoxLayout()
        t_lbl = QtWidgets.QLabel("Throttle")
        t_lbl.setStyleSheet("color: #ff55ff; font-size: 9pt;")
        t_lbl.setAlignment(QtCore.Qt.AlignCenter)
        throttle_vbox.addWidget(t_lbl)

        self.throttle_bar = QtWidgets.QProgressBar()
        self.throttle_bar.setOrientation(QtCore.Qt.Vertical)
        self.throttle_bar.setRange(1000, 2000)
        self.throttle_bar.setValue(1000)
        self.throttle_bar.setFixedSize(30, 90)
        self.throttle_bar.setStyleSheet("QProgressBar { background: #1a212b; border: 1px solid #2a3442; } QProgressBar::chunk { background: #ff55ff; }")
        throttle_vbox.addWidget(self.throttle_bar, 0, QtCore.Qt.AlignCenter)

        self.throttle_val_lbl = QtWidgets.QLabel("1000 Thr")
        self.throttle_val_lbl.setStyleSheet("font-size: 8pt;")
        self.throttle_val_lbl.setAlignment(QtCore.Qt.AlignCenter)
        throttle_vbox.addWidget(self.throttle_val_lbl)

        h_layout_bars.addLayout(throttle_vbox)

        motors_layout.addLayout(h_layout_bars)
        main_layout.addWidget(group_motors)

        # 5. Graphique Ultra-Rapide (PyQtGraph)
        group_graph = QtWidgets.QGroupBox(" Historique Angles (°) ")
        group_graph.setStyleSheet("QGroupBox { color: #00d2ff; font-weight: bold; border: 1px solid #1a212b; margin-top: 5px; } QGroupBox::title { subcontrol-position: top left; padding: 0 3px; }")
        graph_layout = QtWidgets.QVBoxLayout(group_graph)

        radio_layout = QtWidgets.QHBoxLayout()
        self.radio_both = QtWidgets.QRadioButton("Les deux")
        self.radio_roll = QtWidgets.QRadioButton("Roulis seul")
        self.radio_pitch = QtWidgets.QRadioButton("Tangage seul")
        self.radio_both.setChecked(True)
        self.radio_both.setStyleSheet("color: #ffffff;")
        self.radio_roll.setStyleSheet("color: #00ff66;")
        self.radio_pitch.setStyleSheet("color: #00d2ff;")
        
        self.radio_group = QtWidgets.QButtonGroup(self)
        self.radio_group.addButton(self.radio_both)
        self.radio_group.addButton(self.radio_roll)
        self.radio_group.addButton(self.radio_pitch)
        
        radio_layout.addWidget(self.radio_both)
        radio_layout.addWidget(self.radio_roll)
        radio_layout.addWidget(self.radio_pitch)
        graph_layout.addLayout(radio_layout)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("#1a212b")
        self.plot_widget.setYRange(-60, 60)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        graph_layout.addWidget(self.plot_widget)
        main_layout.addWidget(group_graph)

        self.curve_roll = self.plot_widget.plot(pen=pg.mkPen(color="#00ff66", width=2), name="Roll")
        self.curve_t_roll = self.plot_widget.plot(pen=pg.mkPen(color="#00ff66", width=1.5, style=QtCore.Qt.DashLine), name="Target Roll")
        self.curve_pitch = self.plot_widget.plot(pen=pg.mkPen(color="#00d2ff", width=2), name="Pitch")
        self.curve_t_pitch = self.plot_widget.plot(pen=pg.mkPen(color="#00d2ff", width=1.5, style=QtCore.Qt.DashLine), name="Target Pitch")

        # 6. Enregistrement CSV
        group_rec = QtWidgets.QGroupBox(" Enregistrement des Données ")
        group_rec.setStyleSheet("QGroupBox { color: #00d2ff; font-weight: bold; border: 1px solid #1a212b; margin-top: 5px; } QGroupBox::title { subcontrol-position: top left; padding: 0 3px; }")
        rec_layout = QtWidgets.QHBoxLayout(group_rec)
        
        rec_layout.addWidget(QtWidgets.QLabel("Nom du fichier :"))
        self.input_filename = QtWidgets.QLineEdit("telemetry_data.csv")
        self.input_filename.setStyleSheet("background: #1a212b; color: #ffffff; padding: 3px; border: 1px solid #2a3442;")
        rec_layout.addWidget(self.input_filename)

        self.btn_rec = QtWidgets.QPushButton("Démarrer l'enregistrement")
        self.btn_rec.setStyleSheet("background-color: #00ff66; color: #12161c; font-weight: bold; padding: 5px;")
        self.btn_rec.clicked.connect(self.toggle_recording)
        rec_layout.addWidget(self.btn_rec)
        main_layout.addWidget(group_rec)

        # 7. Gains PID
        group_pid = QtWidgets.QGroupBox(" Gains PID ")
        group_pid.setStyleSheet("QGroupBox { color: #00d2ff; font-weight: bold; border: 1px solid #1a212b; margin-top: 5px; } QGroupBox::title { subcontrol-position: top left; padding: 0 3px; }")
        pid_layout = QtWidgets.QGridLayout(group_pid)

        pid_layout.addWidget(QtWidgets.QLabel("KP"), 0, 1, alignment=QtCore.Qt.AlignCenter)
        pid_layout.addWidget(QtWidgets.QLabel("KI"), 0, 2, alignment=QtCore.Qt.AlignCenter)
        pid_layout.addWidget(QtWidgets.QLabel("KD"), 0, 3, alignment=QtCore.Qt.AlignCenter)

        self.pid_inputs = {}
        for r_idx, (axis_lbl, axis_key) in enumerate([("Roll", "roll"), ("Pitch", "pitch"), ("Alt", "alt")]):
            lbl = QtWidgets.QLabel(f"{axis_lbl}:")
            lbl.setStyleSheet("color: #ffcc00; font-weight: bold;")
            pid_layout.addWidget(lbl, r_idx+1, 0)
            
            for c_idx, param in enumerate(["kp", "ki", "kd"]):
                inp = QtWidgets.QLineEdit(str(self.pid_gains[axis_key][param]))
                inp.setStyleSheet("background: #1a212b; color: #ffffff; padding: 3px; border: 1px solid #2a3442;")
                pid_layout.addWidget(inp, r_idx+1, c_idx+1)
                self.pid_inputs[(axis_key, param)] = inp

        btn_apply = QtWidgets.QPushButton("Appliquer les PID")
        btn_apply.setStyleSheet("background-color: #00ff66; color: #12161c; font-weight: bold; padding: 6px;")
        btn_apply.clicked.connect(self.apply_pid_values)
        pid_layout.addWidget(btn_apply, 4, 0, 1, 4)
        main_layout.addWidget(group_pid)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_ui_loop)
        self.timer.start(25)

    def toggle_connection(self):
        if not self.is_connected:
            try:
                self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
                self.is_connected = True
                self.btn_connect.setText("Déconnexion")
                self.btn_connect.setStyleSheet("background-color: #ff4d4d; color: #ffffff; font-weight: bold; padding: 6px;")
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Erreur", f"Connexion impossible sur {self.port}:\n{e}")
        else:
            if self.is_recording:
                self.toggle_recording()
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.is_connected = False
            self.btn_connect.setText("Connexion")
            self.btn_connect.setStyleSheet("background-color: #00ff66; color: #12161c; font-weight: bold; padding: 6px;")

    def toggle_recording(self):
        if not self.is_recording:
            filename = self.input_filename.text().strip() or "telemetry_data.csv"
            if not filename.endswith(".csv"):
                filename += ".csv"
            try:
                self.csv_file = open(filename, mode='w', newline='', encoding='utf-8')
                self.csv_writer = None
                self.is_recording = True
                self.btn_rec.setText("Arrêter l'enregistrement")
                self.btn_rec.setStyleSheet("background-color: #ff4d4d; color: #ffffff; font-weight: bold; padding: 5px;")
                self.input_filename.setEnabled(False)
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Erreur Fichier", f"Impossible d'ouvrir le fichier:\n{e}")
        else:
            self.is_recording = False
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
            self.csv_writer = None
            self.btn_rec.setText("Démarrer l'enregistrement")
            self.btn_rec.setStyleSheet("background-color: #00ff66; color: #12161c; font-weight: bold; padding: 5px;")
            self.input_filename.setEnabled(True)

    def start_serial_thread(self):
        def reader():
            # Format correspondant aux 28 octets (ajusté selon votre structure)
            FORMAT = "<BhhhhhhhhhhfB"
            PACKET_SIZE = struct.calcsize(FORMAT) # Devrait être 28 ou 29 octets
            
            buffer = bytearray()
            
            while True:
                if self.is_connected and self.serial_conn and self.serial_conn.is_open:
                    try:
                        incoming = self.serial_conn.read(self.serial_conn.in_waiting or 1)
                        if incoming:
                            buffer.extend(incoming)
                        
                        while len(buffer) >= PACKET_SIZE:
                            # 1. Chercher l'en-tête 0xCC
                            if buffer[0] != 0xCC:
                                buffer.pop(0)
                                continue
                            
                            packet_bytes = buffer[:PACKET_SIZE]
                            
                            # --- TEMPORAIRE : On désactive le blocage par checksum pour tester ---
                            unpacked = struct.unpack(FORMAT, packet_bytes)
                            data_dict = {
                                "roll": unpacked[1] / 100.0,
                                "pitch": unpacked[2] / 100.0,
                                "target_roll": unpacked[3] / 100.0,
                                "target_pitch": unpacked[4] / 100.0,
                                "throttle": float(unpacked[5]),
                                "escA": float(unpacked[6]),
                                "escB": float(unpacked[7]),
                                "escC": float(unpacked[8]),
                                "escD": float(unpacked[9]),
                                "battery": unpacked[10]/100 * 3.2,
                                "tps_boucle": unpacked[11],                                
                                "armed": unpacked[12],
                            }
                            self.parse_binary_data(data_dict)
                            
                            # Supprimer le paquet traité du buffer
                            del buffer[:PACKET_SIZE]
                                
                    except Exception as e:
                        print(f"Erreur de lecture: {e}")
                else:
                    QtCore.QThread.msleep(10)
                    
        t = threading.Thread(target=reader, daemon=True)
        t.start()

    def parse_binary_data(self, data_dict):
        try:
            # Enregistrement CSV
            if self.is_recording and self.csv_file:
                row = {
                    "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                    "roll": data_dict.get("roll", ""),
                    "pitch": data_dict.get("pitch", ""),
                    "target_roll": data_dict.get("target_roll", ""),
                    "target_pitch": data_dict.get("target_pitch", ""),
                    "escA": data_dict.get("escA", ""),
                    "escB": data_dict.get("escB", ""),
                    "escC": data_dict.get("escC", ""),
                    "escD": data_dict.get("escD", ""),
                    "throttle": data_dict.get("throttle", "")
                }
                if self.csv_writer is None:
                    self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=list(row.keys()))
                    self.csv_writer.writeheader()
                self.csv_writer.writerow(row)
                self.csv_file.flush()

            # Mise à jour des variables internes pour l'interface
            if 'roll' in data_dict:
                self.hist_roll.pop(0)
                self.hist_roll.append(data_dict['roll'])
            if 'pitch' in data_dict:
                self.hist_pitch.pop(0)
                self.hist_pitch.append(data_dict['pitch'])
            if 'target_roll' in data_dict:
                self.hist_target_roll.pop(0)
                self.hist_target_roll.append(data_dict['target_roll'])
            if 'target_pitch' in data_dict:
                self.hist_target_pitch.pop(0)
                self.hist_target_pitch.append(data_dict['target_pitch'])

            if 'armed' in data_dict:
                self.drone_demarre = bool(data_dict['armed'])

            if 'escA' in data_dict: self.esc_A = data_dict['escA']
            if 'escB' in data_dict: self.esc_B = data_dict['escB']
            if 'escC' in data_dict: self.esc_C = data_dict['escC']
            if 'escD' in data_dict: self.esc_D = data_dict['escD']
            if "lx" in  data_dict: self.lx = data_dict['lx']
            if "ly" in  data_dict: self.ly = data_dict['ly']
            if "rx" in  data_dict: self.rx = data_dict['rx']
            if "ry" in  data_dict: self.ry = data_dict['ry']
            if 'throttle' in data_dict: self.throttle = data_dict['throttle']
            if 'tps_boucle' in data_dict: self.tps_boucle = data_dict['tps_boucle']
            if 'battery' in data_dict: self.battery_voltage = data_dict['battery']
        except Exception:
            pass

    def apply_pid_values(self):
        for (axis, param), inp in self.pid_inputs.items():
            try:
                val = float(inp.text().replace(",", "."))
                self.pid_gains[axis][param] = max(0.0, val)
            except ValueError:
                pass
        
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                for axis in ["roll", "pitch", "alt"]:
                    kp, ki, kd = self.pid_gains[axis]["kp"], self.pid_gains[axis]["ki"], self.pid_gains[axis]["kd"]
                    self.serial_conn.write(f"PID:{axis}:{kp},{ki},{kd}\n".encode('utf-8'))
                QtWidgets.QMessageBox.information(self, "Succès", "Gains PID envoyés avec succès !")
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Erreur", f"Erreur d'envoi:\n{e}")
        else:
            QtWidgets.QMessageBox.warning(self, "Attention", "Port série non connecté !")

    def update_ui_loop(self):
        if self.is_connected:
            if self.drone_demarre:
                self.lbl_status.setText("MOTORS: ONLINE")
                self.lbl_status.setStyleSheet("font-size: 14pt; font-weight: bold; color: #00ff66;")
            else:
                self.lbl_status.setText("MOTORS: ARMED / PAUSED")
                self.lbl_status.setStyleSheet("font-size: 14pt; font-weight: bold; color: #ff4d4d;")
        else:
            self.lbl_status.setText("MOTORS: OFFLINE")
            self.lbl_status.setStyleSheet("font-size: 14pt; font-weight: bold; color: #ff4d4d;")

        vx, vy, vz = self.vitesse
        self.lbl_vitesse.setText(f"Vx: {vx:+.2f} m/s | Vy: {vy:+.2f} m/s | Vz: {vz:+.2f} m/s | ||V||: {self.norme_vitesse:.2f} m/s")
        self.lbl_joystick.setText(f"Joystick Gauche  -> LX: {self.lx:+.2f} | LY: {self.ly:+.2f}\nJoystick Droit   -> RX: {self.rx:+.2f} | RY: {self.ry:+.2f}")
        
        tps_ms = self.tps_boucle / 1000.0
        self.lbl_tps_boucle.setText(f"Temps de boucle: {tps_ms:.2f} ms")
        self.lbl_battery.setText(f"Batterie: {self.battery_voltage:.2f} V")

        # Mise à jour des moteurs
        esc_vals = {"A": self.esc_A, "B": self.esc_B, "C": self.esc_C, "D": self.esc_D}
        for m_name, val in esc_vals.items():
            v_int = int(val)
            bar, val_lbl = self.motor_bars[m_name]
            bar.setValue(v_int)
            val_lbl.setText(f"{v_int} ESC")

        # Mise à jour de la barre de Throttle
        t_int = int(self.throttle)
        self.throttle_bar.setValue(t_int)
        self.throttle_val_lbl.setText(f"{t_int} Thr")

        show_roll = self.radio_both.isChecked() or self.radio_roll.isChecked()
        show_pitch = self.radio_both.isChecked() or self.radio_pitch.isChecked()

        self.curve_roll.setData(self.time_data, self.hist_roll if show_roll else [])
        self.curve_t_roll.setData(self.time_data, self.hist_target_roll if show_roll else [])
        self.curve_pitch.setData(self.time_data, self.hist_pitch if show_pitch else [])
        self.curve_t_pitch.setData(self.time_data, self.hist_target_pitch if show_pitch else [])

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = SerialTelemetryApp(port="COM17", baudrate=115200)
    window.show()
    sys.exit(app.exec_())