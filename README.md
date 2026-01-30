# HaloSafe: ISO 15066 Safety Controller

**HaloSafe** is a real-time safety monitoring layer designed for collaborative robots (cobots). It implements **Speed and Separation Monitoring (SSM)** according to ISO/TS 15066 standards, ensuring the robot dynamically reduces speed or stops completely when a human operator enters its workspace.

## 🛡️ Core Logic
HaloSafe operates as a "Man-in-the-Middle" between the motion planner and the robot hardware.

1.  **Admittance Control (The Physics):**
    * Simulates a **Mass-Spring-Damper** system ($M\ddot{x} + D\dot{x} + Kx = F_{ext}$).
    * This gives the robot "virtual weight," making it compliant and safe for physical interaction rather than stiff and dangerous.

2.  **Safety Monitor (The Watchdog):**
    * Continuously calculates the Euclidean distance between the Robot TCP (Tool Center Point) and the Human operator.
    * **Green Zone (> 2.0m):** 100% Velocity.
    * **Yellow Zone (0.5m - 2.0m):** Linear velocity scaling.
    * **Red Zone (< 0.5m):** Protective Stop (0% Velocity).

## 🚀 Usage
You can run the visualization dashboard to see the safety logic in action:

```bash
python safety_controller.py

```

**Output:**
The script launches a live ASCII dashboard showing:

* **Human Distance:** Simulated proximity of the operator.
* **Velocity Scale:** The calculated safety factor (0% - 100%).
* **Robot Velocity:** The final safe output command sent to the motors.

## 🛠️ Configuration

* **Mass (M):** 5.0 kg (Virtual inertia)
* **Damping (D):** 10.0 Ns/m (Resistance to motion)
* **Protective Stop Distance:** 0.5m

## 📦 Dependencies

* Python 3.10+
* NumPy

**Author:** Charles Austin
*Focus: Robotics Safety Systems, Human-Robot Interaction (HRI)*
