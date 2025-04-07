# MIMO-Control-Design-Optimization-with-PI-and-LQR

This project involves designing and tuning two advanced controllers—MIMO PI (Proportional-Integral) and LQR (Linear Quadratic Regulator)—for a stable, controllable plant system with three states, inputs, and outputs. The goal is to achieve the fastest possible response while minimizing overshoots and adhering to input constraints (|u| ≤ 10). The system is analyzed under nominal conditions and model mismatch scenarios to evaluate robustness.

Key Components
1. System Analysis
- Stability & Controllability: The plant is asymptotically stable (all eigenvalues have negative real parts) and fully controllable (full-rank controllability matrix).

- Open-Loop Response: States converge to zero with oscillations due to complex eigenvalues.

2. Controller Designs
-MIMO PI Controller:

--Integral Control (Ki): Designed as Ki = inv(Ks), where Ks = -C * inv(A) * B. Tuned empirically to balance speed and overshoot.

--Proportional Control (Kp): Added to reduce steady-state error. Tuning parameter b prioritized over a (integral) for faster response.

--Performance: Faster than open-loop but exhibits slight overshoot.

-LQR Controller:

--Optimal Feedback: Minimizes cost function J = ∫(xᵀQx + uᵀRu) dt.

--Tuning: Diagonal matrices Q (state penalties) and R (input effort) adjusted to meet input constraints.

--Performance: Slower than PI but significantly reduces overshoots.

3. Model Mismatch Robustness
-Both controllers tested under perturbed dynamics (A → 0.8A).

-Results: PI and LQR responses remain stable with minimal performance degradation.

4. Comparative Analysis


Repository Contents
- Report: Detailed explanation of controller design, tuning, and simulation results.
- MATLAB/Simulink Files
- Figures: Plots of state trajectories, input/output responses, and robustness tests.

Dependencies
-MATLAB (for control design and simulation)

-Control System Toolbox (for LQR and state-space analysis)

How to Use
Clone the repository.

Open MATLAB and run the provided scripts to:

Simulate open-loop and closed-loop responses.

Tune PI (a, b) and LQR (Q, R) parameters.

Test robustness by modifying the system matrix (A).

Refer to the report for theoretical insights and result interpretations.

Key Results
-PI Controller: Best for rapid response but trades off overshoots.

-LQR Controller: Optimal for stability and constraint adherence.

-Robustness: Both controllers perform well under model uncertainties.
