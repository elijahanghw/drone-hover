# drone-hover

Compute the hovering capabilities of drones with arbitrary configurations.

**Updates**:

[20 May 2024]
1. Motor commands are now proportionate to the square of motor RPM.
2. Propeller forces are now defined using force and moment constants rather than maximum thrust and moments.

## Defining drone bodies
The drone has a body-fixed coordinate system attached to the C.G. which follows the right-handed convention ($x$ axis pointing to the front, $y$ axis pointing to the right, and $z$ axis pointing down).

Drones are defined using classes, and require inertia and propeller properties as class variables.

Inertia properties are the mass and moment of inertia of the drone, and are defined using variables.

Propeller properties are defined using dictionaries, and require the following keywords:

`"loc":[x,y,z]`: List that defines the $(x,y,z)$ coordinates of the propeller in body-fixed axis.

`"dir":[x,y,z,r]`: List that defines the direction of thrust and rotation for the propeller. Includes 4 numbers, first 3 numbers are the $(x,y,z)$ vector defining the thrust direction, and the last number $r=1$ for CCW rotation or $r=-1$ for CW rotation (as viewed from the top of the propeller). Direction $(x,y,z)$ does not need to be unit vector as the optimizer will scale it automatically.

`"constants":[k_f, k_m]`: Thrust and moment constant for the propeller.

`"wmax": wmax`: Maximum rotational speed (rad/s) for propeller.

Example: 

    self.props = [{"loc":[1, 1, 0], "dir": [0, 0, -1, 1], "constants": [4.5e-04, 2.5e-06], "wmax": 30000},
                  {"loc":[-1, 1, 0], "dir": [0, 0, -1, -1], "constants": [4.5e-04, 2.5e-06], "wmax": 30000},
                  {"loc":[-1, -1, 0], "dir": [0, 0, -1, 1], "constants": [4.5e-04, 2.5e-06], "wmax": 30000},
                  {"loc":[1, -1 0], "dir": [0, 0, -1, -1], "constants": [4.5e-04, 2.5e-06], "wmax": 30000}]

There are 2 ways to define the drone body.
1. Creating a class that follows the format as seen in `drone_hover.standard_bodies`.
2. Call the `Custombody` object

Example:

    from drone_hover.custom_bodies import Custombody
    drone = Custombody(mass, Ix, Iy, Iz, Ixy, Ixz, Iyz, props)

## Optimization

Optimization is performed using `scipy.optimize.minimize` module, using the SLSQP algorithm.

Propeller angular velocity is defined as $\omega = \omega_{max} u$, where $u \in [0.02,1]$.

Currently, the command bounds are arbitrarily limited to 0.02 to 1, where 0.02 corresponds to idle RPM, and 1 corresponds to maximum RPM. This means that $\omega_{min} = 0$. This is done to remove bilinear terms when taking the square of propeller angular rotation for convinience.

## Current capabilities: 

- Determine whether a drone can hover statically, while spinning, or not able to hover at all.
- Works on drones with arbitrary configurations (e.g. number of propellers, location of propellers, direction of propellers, etc.).
- Computes the input commands for most efficient hover.
- Computes the cost of most efficient hover.

## Limitations:

- Spinning hover optimization does not work when force is aligned with torque for all values of input commands. SLSQP require constraints to be twice differentiable. To consider alternative optimization algorithms.
- The origin of the body-fixed coordinates is assumed to always be on the C.G. To consider a more general case where the origin and the C.G. do not coincide.
