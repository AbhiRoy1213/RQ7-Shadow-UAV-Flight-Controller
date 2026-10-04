# RQ-7 Shadow UAV: Digital Flight Controller & Hardware Validation

This repository contains the MATLAB and Simulink models for a custom Guidance, Navigation, and Control (GNC) system designed for the RQ-7 Shadow UAV. The project bridges the gap between continuous theoretical mathematics and real-world embedded systems by transitioning a theoretical flight controller into a discrete 50 Hz hardware environment. It successfully demonstrates how robust custom control laws can keep an aircraft stable when confronting mechanical constraints, non-linear aerodynamics, and dynamic disturbance torques.

## System Architecture & Hardware Constraints

The simulation replaces idealized actuators with physically accurate models to validate the control loops against the RQ-7 Shadow's actual mechanical limits.

| Component | Specification | Operational Constraint |
| :--- | :--- | :--- |
| **Clock Speed** | 50 Hz (0.02s Sample Time) | Defines the discrete Tustin integration rate |
| **Actuator Saturation** | $\pm 25^\circ$ (0.43 rad) | Maximum physical elevator/aileron deflection |
| **Servo Slew Rate** | $60^\circ/\text{s}$ (1.04 rad/s) | Maximum mechanical rotational speed |
| **Aerodynamic Plant** | LTI Systems ($G_h$, $G_\phi$) | Extracted from continuous MATLAB 6DOF linearizations |

## Control Strategy & Signal Routing

The flight controller isolates the longitudinal and lateral dynamics into two decoupled digital feedback loops.

*   **Altitude (Longitudinal) Loop:** A discrete Proportional-Derivative (PD) controller manages the elevator. The derivative term provides critical damping to arrest pitch oscillations during aggressive climbs.
*   **Bank Angle (Lateral) Loop:** A discrete Proportional-Integral (PI) controller manages the ailerons. The integrator eliminates steady-state error, forcing the airframe to perfectly lock onto commanded bank angles despite natural aerodynamic roll subsidence.

> **Note:**  
> <img width="708" height="179" alt="Screenshot 2026-10-04 002401" src="https://github.com/user-attachments/assets/0d4df082-56a1-4415-bdb5-8633d834edde" />


## Key Engineering Solutions: The Digital Flight Director

Transitioning the perfect math to hardware constraints immediately introduced command saturation. Instantaneous pilot step inputs forced the PD controller to demand physically impossible maneuvers, causing the simulated servos to brick-wall at 25 degrees and resulting in severe altitude ballooning.

To resolve this without sacrificing controller aggressiveness, a **Digital Flight Director** was engineered by implementing command-path rate limiters:
*   **Altitude Command Limiter:** Restricts instantaneous altitude requests to a steady **0.5 m/s** climb rate ramp.
*   **Roll Command Limiter:** Restricts instantaneous bank angle requests to a smooth **5°/s** roll rate.

By smoothing the incoming error signal, the 170 kg airframe is able to track the 10-meter climb and 15-degree bank targets smoothly without maxing out the mechanical servos or destabilizing the plant.

## Simulation Results & Telemetry

The flight telemetry proves the discrete 50 Hz controllers can successfully command the airframe through simultaneous longitudinal and lateral maneuvers within a 30-second flight envelope, strictly obeying all hardware slew rates.

> **Note:**  
> <img width="411" height="269" alt="Screenshot 2026-10-04 002507" src="https://github.com/user-attachments/assets/5e2de4d2-d01c-4747-9468-65b02e1299ed" />
> <img width="408" height="264" alt="Screenshot 2026-10-04 002529" src="https://github.com/user-attachments/assets/0d95aeba-d2ff-44cd-bdf8-00c489c45c37" />



**Technologies Used:** MATLAB, Simulink, Control System Toolbox, Discrete Control Theory.
