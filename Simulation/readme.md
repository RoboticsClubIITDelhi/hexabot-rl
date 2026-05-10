# Hexapod v1: Simulation & Hardware Control

This repository contains the simulation, inverse kinematics (IK) engine, and hardware communication stack for the 1.5kg hexapod project. The architecture relies on a PyBullet digital twin that calculates trajectories and IK in real-time, streaming the resulting joint angles to an ESP32, which then drives the SC15 serial bus servos.

## System Architecture

1. **Python Control Node (`script.py` & `script3.py`):** Runs the PyBullet physical simulation. It calculates 3-DOF inverse kinematics for each leg, generates Bezier curves for swing phases and linear paths for stance phases, and packages the data.
2. **Communication Link:** Commands are sent over a Serial COM port at 230400 baud. The ESP32 listens via a paired Bluetooth Serial connection. 
3. **ESP32 Firmware (`esp_code.ino`):** Acts as a high-speed bridge. It parses the incoming packet format (`<count, id1, pos1, id2, pos2...>`) and executes a simultaneous `SyncWritePos` to all target SC15 servos.

---

## File Overview

### 1. `esp_code.ino` (Hardware Firmware)
The C++ code for the ESP32. It initializes a Bluetooth Serial connection (`"Hexapod_BT"`) and a hardware serial connection to the SC15 servo board at 1,000,000 baud. It constantly polls the Bluetooth buffer with a fast 2ms timeout, parses incoming packets, and commands the servos to move to their new positions in 15ms intervals, ensuring smooth, synchronized motion.

### 2. `script.py` (Basic Tripod Gait)
A standalone test script for basic locomotion. 
* Sets up the PyBullet physics environment and loads `robot.urdf`.
* Calculates neutral standing positions and steps the robot through a continuous, pre-calculated tripod gait (alternating groups of 3 legs).
* Good for testing basic servo mapping, sign conventions, and ensuring the robot can stand and walk forward without input.

### 3. `script3.py` (Interactive WASD Controller)
The primary operational script. It expands upon the basic script by introducing keyboard-driven omnidirectional control.
* **Pre-calculated Arrays:** Generates Bezier and linear point arrays for both walking (forward/backward) and turning (sweeping sideways).
* **Friction Tuning:** Adjusts lateral friction within PyBullet so the digital twin accurately mimics the rotational grip needed to turn the physical chassis.
* **The "Matrix" Logic:** Reads the trajectory arrays either forward or backward in real-time depending on whether you are pressing `W`, `A`, `S`, or `D`, allowing seamless transitions between walking and rotating.

---

## How It Works: The Math

* **Inverse Kinematics (IK):** The `IK(tx, ty, tz)` function takes a target foot coordinate in 3D space (relative to the leg mount) and calculates the required Coxa (yaw), Femur (pitch), and Tibia (pitch) angles using trigonometry (Law of Cosines).
* **Trajectory Generation:** * **Swing Phase:** A quadratic Bézier curve lifts the foot off the ground, moves it to the new position, and places it down softly.
  * **Stance Phase:** A linear interpolation drags the foot backward relative to the body, which physically pushes the hexapod forward.
* **Angle Translation:** PyBullet operates in radians centered around a neutral pose. The physical SC15 servos operate on a 0-1023 step range (centered at 512). The Python scripts handle this conversion and apply specific hardware sign reversals (`HW_SIGNS`) to account for physical mounting orientations.

---

## How to Start and Run

### Prerequisites
* **Hardware:** ESP32 flashed with `esp_code.ino`, powered SC15 servos.
* **Software:** Python 3.x, `pybullet`, `numpy`, `pyserial`.
* **Assets:** Ensure `robot.urdf` and `plane.urdf` are in the same directory as the Python scripts.

### Step 1: Flash the ESP32
1. Open `esp_code.ino` in the Arduino IDE.
2. Ensure you have the `SCServo` library installed.
3. Flash the code to the ESP32.

### Step 2: Establish Connection
1. Power on the Hexapod (ensure the ESP32 and servos have sufficient power).
2. Pair your computer with the Bluetooth device named **"Hexapod_BT"**.
3. Identify the COM port assigned to this Bluetooth connection (e.g., `COM9` on Windows or `/dev/rfcomm0` on Ubuntu).
4. *Important:* Update the `Serial('COM9', 230400...)` line in your Python scripts to match your active port.

### Step 3: Run the Simulation & Control
To run the interactive controller:
```bash
python script3.py