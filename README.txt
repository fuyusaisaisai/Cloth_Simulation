CIS 563 Assignment 2: Cloth Simulation -- a Position Based Dynamics Approach

1. Compare to the Jello simluation, what are the advantages of using PBD over using mass-spring system?

In the Jello simulation, we use Newton's Law by calculate velocity from acceleration and then calculate position from velocity. However, PBD allows us to control over position directly with explicit integration. It removes typical instablity problems, allowing us to handle constraints concerning positions and the solver is easy to understand.

2. What is the function of the stiffness for each constraint? What value will you use for simulating a rigid body?

Stiffness can prevent the system from sudden changes and provide a linear relation ship between the error and correction. I would use 0.9~1.0 as a value for rigid body.

3. The collision detection system used in PBD combined both continuous collision and static collision detection. Explain in short the difference between these two mechanisms. Which mechanism is used in Jello simulation(detecting, not resolving)?

Continuous collision is when the predicted position is in collide the an obstable (within the threshold space) while the current position has not yet collided with it. Static collision is when current position is already colliding with the obstacle. Static collistion is used in Jello simulation.

4. Does your stretch constraint resolving preserve momentum? What about fixed point constraint and collision constraint? Why?

My stretch constraint resolves preserve momentum because it's internal constraints.
Fixed point constraint and collistion constraints don't resolve preserve momentum because they are external constraints and external forces will definitely change the momentum.

5. Which part of the PBD concept cost you most of time in understanding? Which part cost most in implementing?

Inequality constraint cost me most of time in understanding. The line intersection part cost most in implementing.


Discussing the project 
Firstly, external forces(gravity) is applied to each particle using Newton's Law and predicted velocities are calculated and damped. Then predited position is also calulated based on the velocity. The main task is to implement constraint detection and constraint projection. The former can be done by considering whether certain constraint is satisfied. For inequality constraints, constraint list is "pushed back" when constraint value is below zero(such as an intersection with obstacle). For equality constraints, as "equal" can hardly be realized, this kind of constraints always exists(such as stretch constrains for each edge, fixed points for certain points and bending constraints for each pair of neighboring triangles). Then in projection part, gradient of constraints over each particle position is calculated and added to the current position with tuned stiffness. Then velocities are calculated from current and predicted positions. Lastly restitution and norm_fraction are considered if a particle is in collision(contact with an obstacle). 

I tried the real bending constraint and it works well. For each two pairs of neighboring trianles, I calculated their normal vector by cross multiply the edges in certain order. Then the angle between these two are calculated(same with the angle of the normal vectors). Constraint is formed based on this angle. Then according to Appendix A part of the paper*, I finished the real bend constraint projection.

I also tried "Add other primitives into the scene". I intended to add a box shape. However once I add box line in the xml file, the scene will disappear.

I also tried Self collision. I didn't come up with a way to test it.












