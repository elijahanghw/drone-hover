# drone-hover

Compute the hovering capabilities of drones with arbitrary configurations.

**Updates**:

[20 May 2024]
1. Motor commands are now proportionate to the square of motor RPM.
2. Propeller forces are now defined using force and moment constants rather than maximum thrust and moments.

## Defining drone bodies
The drone has a body-fixed coordinate system which follows the right-handed convention ($x$ axis pointing to the front, $y$ axis pointing to the right, and $z$ axis pointing down). Propeller positions and directions are defined using this coordinate system. The C.G. of the drone may not necessarily coincide with the origin of the coordinate system, and needs to be defined.

Drones are defined using classes, and require inertia and propeller properties as class variables.

Inertia properties are the mass and moment of inertia of the drone, and are defined using variables. C.G. location is also defined as a list.

Propeller properties are defined using dictionaries, and require the following keywords:

`"loc":[x,y,z]`: List that defines the $(x,y,z)$ coordinates of the propeller in body-fixed axis.

`"dir":[x,y,z,r]`: List that defines the direction of thrust and rotation for the propeller. Includes 4 numbers, first 3 numbers are the $(x,y,z)$ vector defining the thrust direction, and the entry indicates counterclockwise (ccw) or clockwise (cw) rotation (as viewed from the top of the propeller). Direction $(x,y,z)$ does not need to be unit vector as the optimizer will scale it automatically.

`"constants":[k_f, k_m]`: Thrust and moment constant for the propeller.

`"wmax": wmax`: Maximum rotational speed (rad/s) for propeller.

Example: 

    props = [{"loc":[1, 1, 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
             {"loc":[-1, 1, 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
             {"loc":[-1, -1, 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
             {"loc":[1, -1 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927}]

There are 2 ways to define the drone body.
1. Creating a class that follows the format as seen in `drone_hover.standard_bodies`.
2. Call the `Custombody` object

Example:

    from drone_hover.custom_bodies import Custombody
    drone = Custombody(self, mass, cg, Ix, Iy, Iz, Ixy, Ixz, Iyz, props)

## Propeller Commands

This code utilizes 2 levels of mapping for the propeller commands.
1. The propeller angular velocity is normalized such that $f:\omega \rightarrow \hat{\omega}$, where $\omega \in [0.02\omega_{max}, \omega_{max}]$ and $\hat{\omega} \in [0.02, 1]$. The factor 0.02 is arbitrarily selected to be the idling speed of the propeller. This mapping embeds the propeller information into the propeller effectiveness matrices. 
2. When giving actual commands to the drone, it is more convinient to give a command $u \in [0,1]$. Hence, a second map $g:\hat{\omega} \rightarrow u$ is defined.

This is done to ensure that the equations remain linear (to $\omega^2$). Optimization will be performed using $\hat{\omega}$, while actual controls will be performed using $u$.

## Optimization

Optimization is performed using `scipy.optimize.minimize` module, using the SLSQP algorithm.

## Current capabilities: 

- Determine whether a drone can hover statically, while spinning, or not able to hover at all.
- Works on drones with arbitrary configurations (e.g. number of propellers, location of propellers, direction of propellers, etc.).
- Computes the input commands for most efficient hover.
- Computes the maximum thrust to weight ratio at hovering configuration
- Computes the cost of most efficient hover.

## Limitations:

- Spinning hover optimization does not work when force is aligned with torque for all values of input commands. SLSQP require constraints to be twice differentiable. To consider alternative optimization algorithms.
