# HaloSafe: ISO 15066 Compliant Admittance Controller

![Python](https://img.shields.io/badge/Python-3.10%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Safety_Level-ISO_15066-red)

**HaloSafe** is a safety-critical control loop designed for Collaborative Robots (Cobots). It implements a two-layer safety architecture that combines **Admittance Control** (Mass-Spring-Damper compliance) with **Speed & Separation Monitoring** (SSM) as defined in ISO/TS 15066.

## 🎯 Project Goal
Demonstrate how raw sensor data is transformed into safe robotic actuation using differential equations and real-time state monitoring, without relying on "black box" motion libraries.

## 🏗️ Architecture

The system operates on a **240Hz control loop** with two distinct layers:

### Layer 1: The "Muscle" (Admittance Control)
Instead of stiff position control, the robot acts as a dynamic mass-spring-damper system. This allows it to "yield" to external forces (human contact).

$$M \ddot{x} + D \dot{x} + K(x - x_{ref}) = F_{ext}$$

* **M (Virtual Mass):** Inertia of the robot (Tunable).
* **D (Virtual Damping):** Viscosity to prevent oscillation.
* **F_ext:** External force vector from the environment.

### Layer 2: The "Skin" (Safety Monitor)
A supervisory layer that overrides the velocity commands based on human proximity.
* **Zone 1 (Free Space):** > 2.0m → 100% Velocity
* **Zone 2 (Dynamic Scaling):** 0.5m - 2.0m → Linear Velocity Scaling
* **Zone 3 (Protective Stop):** < 0.5m → 0% Velocity (Hard Stop)

## 🚀 Usage

### 1. Dependencies
No heavy ROS installation required. Uses pure NumPy for solver logic.
```bash
pip install numpy
