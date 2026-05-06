# 2D Quadrotor Control Barrier Function (CBF-QP)

MATLAB implementation of a Control Barrier Function - Quadratic Program (CBF-QP) safety filter for a planar (2D) quadrotor. This repository contains the simulation environment and control architecture developed for MMAE 549 under Professor Ersin Das.

The project demonstrates how a CBF-QP acts as an active safety filter, minimally modifying the control inputs of a baseline nominal Proportional-Derivative (PD) controller to guarantee obstacle avoidance and satisfy physical actuator limits.

## Repository Structure

This repository includes four main MATLAB scripts, each demonstrating a different testing scenario and robustness study:

* **`drone_cbf_qp.m`**
  The default trial and baseline comparison. This script runs the standard simulation, directly comparing the trajectory and control effort of the unconstrained nominal PD controller against the safe CBF-QP filter.

* **`drone_cbf_qp_mass_disturbance.m`**
  A robustness study testing parameter mismatch. This script introduces separate variables for the actual plant mass and the controller's internal simulation mass. It demonstrates how the safety filter adapts when the quadrotor is heavier than the controller expects.

* **`drone_cbf_qp_wind_disturbance.m`**
  A robustness study testing unmodeled external forces. The dynamics function in this script is altered to accept `vw` (vertical wind) and `hw` (horizontal wind) parameters. These act as induced accelerations to test the limits of the safety manifold.

* **`drone_cbf_qp_multiple_obstacled.m`**
  An expanded formulation testing complex environments. This script includes the QP formulation necessary to handle multiple static obstacles simultaneously. Obstacle locations are easily configurable, and the script generates a combined plot showing the CBF inequality values for all obstacles over time.

## Acknowledgments

The baseline CLF-CBF-QP code framework, which served as the architectural foundation for this safety filter, was provided by Professor Ersin Das.
