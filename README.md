# drone-hover

Compute the hovering capabilities of drones with arbitrary configurations.

## Defining drone bodies
The drone has a body-fixed coordinate system attached to the C.G. which follows the right-handed convention (x pointing to the front, y pointing to the right, and z pointing down).

Drones are defined using classes, and require inertia and propeller properties as class variables.

Inertia properties are the mass and moment of inertia of the drone, and are defined using variables.

Propeller properties are defined using dictionaries, and require the following keywords:

`"loc":[x,y,z]`: List that defines the $(x,y,z)$ coordinates of the propeller in body-fixed axis.

`"dir":[x,y,z,r]`: List that defines the direction of thrust and rotation for the propeller. Includes 4 numbers, first 3 numbers are the $(x,y,z)$ vector defining the thrust direction, and the last number $r=1$ for CCW rotation or $r=-1$ for CW rotation (as viewed from the top of the propeller). Direction $(x,y,z)$ does not need to be unit vector as the optimizer will automatically scale it accordingly.

`"force":[f_max, t_max]`: List of maximum thrust and maximum torque from the propeller.

Example: 

    self.props = [{"loc":[1, 1, 0], "dir": [0, 0, 1, 1], "force": [10, 1]},
                  {"loc":[-1, 1, 0], "dir": [0, 0, 1, -1], "force": [10, 1]},
                  {"loc":[-1, -1, 0], "dir": [0, 0, 1, 1], "force": [10, 1]},
                  {"loc":[1, -1 0], "dir": [0, 0, 1, -1], "force": [10, 1]}]

## Optimization

Optimization performed using `scipy.optimize.minimize` module, using the SLSQP algorithm.

Currently, the command bounds are limited to 0.02 to 1, where 0.02 corresponds to idle rotation, and 1 corresponds to maximum thrust.

## Current capabilities: 

- Determine whether a drone can hover statically, while spinning, or not able to hover at all.
- Works on drones with arbitrary configurations (e.g. number of propellers, location of propellers, direction of propellers, etc.).
- Computes the input commands for most efficient hover.
- Computes the cost of most efficient hover.

## Limitations:

- Motor commands are are linearly mapped to force output. To update to be proprotionate to square of propeller angular velocity.
- Spinning hover optimization does not work when force is aligned with torque for all values of input commands. SLSQP require constraints to be twice differentiable. To consider alternative optimization algorithms.
- The origin of the body-fixed coordinates is assumed to always be on the C.G. To consider a more general case where the origin and the C.G. do not coincide.
