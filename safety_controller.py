# safety_controller.py

import numpy as np
import sys
import time
import math
from typing import Tuple, Optional

# --- CORE LOGIC: ADMITTANCE CONTROLLER ---
class AdmittanceController:
    """
    A compliant control loop implementing a Mass-Spring-Damper system for 
    Human-Robot Interaction (HRI).
    
    Implements the control law:
        M * x_dd + D * x_d + K * (x - x_ref) = F_ext
    """

    def __init__(
        self, 
        mass: float = 5.0, 
        damping: float = 10.0, 
        stiffness: float = 0.0, 
        dt: float = 0.05
    ):
        """
        Args:
            mass (float): Virtual mass in kg. Higher = harder to accelerate.
            damping (float): Virtual damping in Ns/m. Higher = more resistance.
            stiffness (float): Virtual stiffness in N/m. 0.0 = free-drive mode.
            dt (float): Control loop cycle time in seconds.
        """
        self.M = mass
        self.D = damping
        self.K = stiffness
        self.dt = dt

        # State Vectors (Position, Velocity)
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)

    def step(self, f_ext: np.ndarray, x_ref: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes the next state of the robot based on external forces.
        """
        if x_ref is None:
            x_ref = self.position

        # 1. Solve Differential Equation for Acceleration (x_dd)
        # x_dd = (F_ext - D*x_d - K*(x - x_ref)) / M
        damping_force = self.D * self.velocity
        spring_force = self.K * (self.position - x_ref)
        
        acceleration = (f_ext - damping_force - spring_force) / self.M

        # 2. Integrate (Euler Method)
        self.velocity += acceleration * self.dt
        self.position += self.velocity * self.dt

        return self.position, self.velocity

# --- SAFETY LAYER: ISO 15066 MONITOR ---
class SafetyMonitor:
    """
    Implements ISO/TS 15066 style Speed and Separation Monitoring (SSM).
    Dynamically scales the robot's allowable velocity based on human proximity.
    """

    def __init__(self, min_dist: float = 0.5, max_dist: float = 2.0):
        self.min_dist = min_dist
        self.max_dist = max_dist

    def get_velocity_scaling_factor(self, robot_pos: np.ndarray, human_pos: np.ndarray) -> float:
        """
        Calculates the safety scaling factor (0.0 to 1.0).
        """
        distance = np.linalg.norm(robot_pos - human_pos)

        # 1. Hard Stop Zone (Protective Stop)
        if distance < self.min_dist:
            return 0.0
        
        # 2. Free Zone (Full Speed)
        if distance > self.max_dist:
            return 1.0
        
        # 3. Dynamic Scaling Zone (Linear Interpolation)
        scale = (distance - self.min_dist) / (self.max_dist - self.min_dist)
        return scale

# --- VISUALIZATION: LIVE DASHBOARD ---
if __name__ == "__main__":
    # Initialize the "Brain"
    controller = AdmittanceController(mass=5.0, damping=10.0)
    safety = SafetyMonitor(min_dist=0.5, max_dist=2.5)

    print("\n--- HALOSAFE: ACTIVE SAFETY MONITOR ---")
    print("  [SYSTEM ONLINE]")
    print("  [MODE: ISO 15066 Speed & Separation Monitoring]")
    print("-" * 75)

    t = 0
    try:
        while True:
            # 1. Simulate Human walking back and forth (Sine wave)
            # Distance oscillates between 0.4m (DANGER) and 3.0m (SAFE)
            human_dist = 1.7 + 1.3 * math.sin(t * 0.4) 
            
            # 2. Get Safety Scale Factor
            # We fake 3D positions for the math: Robot at 0, Human at X
            scale = safety.get_velocity_scaling_factor(
                np.array([0,0,0]), 
                np.array([human_dist, 0, 0])
            )

            # 3. Simulate a continuous Force input (User pushing robot)
            f_input = np.array([15.0, 0.0, 0.0])
            pos, vel = controller.step(f_ext=f_input)
            
            # 4. Apply Safety Scale to Velocity
            safe_vel = vel[0] * scale

            # --- ASCII VISUALIZATION ---
            # Create a progress bar for safety
            bar_len = 25
            filled = int(bar_len * scale)
            bar = "█" * filled + "░" * (bar_len - filled)
            
            # Determine Status Label
            if scale >= 0.9:
                status = "🟢 SAFE"
            elif scale > 0.0:
                status = "🟡 WARN"
            else:
                status = "🔴 STOP"

            # The \r makes it overwrite the same line (Animation effect)
            sys.stdout.write(
                f"\r[HUMAN DIST: {human_dist:.2f}m]  "
                f"[{bar}] {int(scale*100):3d}% MAX SPD  "
                f"| {status} "
                f"| ROBOT VEL: {safe_vel:.3f} m/s"
            )
            sys.stdout.flush()

            time.sleep(0.05)
            t += 0.05

    except KeyboardInterrupt:

        print("\n\n[SYSTEM SHUTDOWN]")
