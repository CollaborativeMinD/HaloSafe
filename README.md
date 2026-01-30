# HaloSafe: ISO 15066 Admittance Control & Safety Monitor

**HaloSafe** is a safety-critical control architecture for collaborative robotics (cobots). It implements a real-time **Admittance Control Loop** coupled with an **ISO/TS 15066 Speed & Separation Monitor**, allowing a robot to dynamically scale its velocity and compliance based on human proximity and external contact forces.

## 📐 Control Architecture

The system executes a 240Hz control loop comprising two primary layers:

### 1. The Admittance Layer (Force Control)
Unlike standard position control, HaloSafe implements an impedance-based control law that renders the robot compliant to external disturbances. The system solves the mass-spring-damper differential equation in real-time:

$$M_d \ddot{x}_d + D_d \dot{x}_d + K_d (x_d - x_{ref}) = F_{ext}$$

Where:
* **$M_d$ (Virtual Mass):** Inertia scaling (kg). Higher values increase resistance to acceleration.
* **$D_d$ (Virtual Damping):** Viscous friction (Ns/m). Critical for preventing oscillation during contact.
* **$F_{ext}$:** External force vector applied by the operator.

### 2. The Safety Layer (Speed & Separation Monitoring)
A supervisory safety layer overrides the admittance output based on the Euclidean distance between the robot's Tool Center Point (TCP) and the operator.

* **Zone 1 (Free Space):** $d > 2.0m$ → **100% Velocity**
* **Zone 2 (Dynamic Scaling):** $0.5m < d < 2.0m$ → **Linear Interpolation**
    * Formula: $V_{scale} = \frac{d - d_{min}}{d_{max} - d_{min}}$
* **Zone 3 (Protective Stop):** $d < 0.5m$ → **0% Velocity (Category 2 Stop)**

## 📊 Live Telemetry Dashboard
The system visualizes the safety state in the terminal for real-time debugging:

```text
[HUMAN DIST: 0.42m]  [░░░░░░░░░░░░░░░░░░░░░░░░░]   0% MAX SPD  | 🔴 STOP | ROBOT VEL: 0.000 m/s
[HUMAN DIST: 1.92m]  [████████████████░░░░░░░░░]  70% MAX SPD  | 🟡 WARN | ROBOT VEL: 1.062 m/s
[HUMAN DIST: 2.62m]  [█████████████████████████] 100% MAX SPD  | 🟢 SAFE | ROBOT VEL: 1.500 m/s

```

## 🛠️ Installation & Usage

### Dependencies

The project uses pure `numpy` for vector math and Euler integration to maintain transparency and portability.

```bash
pip install numpy

```

### Execution

Run the controller to initialize the physics loop and safety monitor:

```bash
python safety_controller.py

```

## 📂 Project Structure

* `safety_controller.py`: Contains the `AdmittanceController` class (physics solver) and `SafetyMonitor` class (ISO logic).
* **Physics Engine:** Custom differential equation solver using Euler integration.

---

**Author:** Charles Austin
