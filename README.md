# drone-hover

Compute the hovering capabilities of drones with arbitrary configurations.

## Current capabilities: 
- Determine whether a drone can hover statically, while spinning, or not able to hover at all.
- Works on drones with arbitrary configurations (e.g. number of propellers, location of propellers, direction of propellers, etc.).
- Computes the input commands for most efficient hover.
- Computes the cost of most efficient hover.

## Limitations:
- Motor commands are are linearly mapped to force output. To update to be proprotionate to square of propeller angular velocity.
- Spinning hover optimization does not work when force is aligned with torque for all values of input commands. SLSQP require constraints to be twice differentiable. To consider alternative optimization algorithms.
