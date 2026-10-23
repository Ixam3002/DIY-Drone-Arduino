# Drone DIY

**The goal of this project** is to design a drone from scratch. It contains the 3D design, electronic design, radio communication and code's developpment to stabilize the drone.

## Realization

### Numerical simulator

First of all, we developped a numerical simulator based on physics equations (Solid Mechanics) to implement all drone's functionalities like the control system and also to test the remote control. This program is in Python's langage and consider all geometrical and caracteristical parameters (like the motor's strength).


### Remote control

We use a Xbox remote to control the drone, an ESP32-S3 is used to decode the GIP protocol. Then, data are transmited to a Arduino Micro by using an UART protocol. Finally, the remote communicate in radio to the drone via the nRF24L01+ module. 

```text
Xbox Remote
      │
      ▼
   ESP32-S3
      │ UART
      ▼
 Arduino Micro
      │
      │ nRF24L01+
      ▼
     Drone
```

### Drone

The components for the embedded system are :

| Components | Function |
|---|---|
| Arduino Micro | Micro controlor |
| MPU6050 | Accelerometer + gyroscope |
| BMP180 | measure the altimeter |
| nRF24L01+ | Radio communication |

### Control system

The control system is based on cascaded PID which consist to :

- an outer loop to calculate the angle error : ωref = Kpangle · (θref - θ)
- an inner loop to consider the rotational speed : u = Kp · eω + Ki · ∫eω dt + Kd · (deω/dt)

The control system scheme is as follow (picture) :




## Conception

The drone's structure is designed on Onshape (link) and print in 3D.

## État du projet

Le projet est actuellement en développement.

Les principaux objectifs sont :

- [ ] Communication radio
- [ ] Lecture des capteurs
- [ ] Stabilisation
- [ ] Finalisation de la structure 3D
- [ ] Assemblage
- [ ] Premiers essais réels

## Documentation

Les détails techniques du projet seront progressivement ajoutés dans le dossier [`documentation/`](documentation/).

## Contributeurs

- **Ixam3002**
- **Thomas127**
