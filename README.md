## fsm-task-exec

Legacy code: addon module for [RobotToolKit](https://github.com/epfl-lasa/robot-toolkit). This package executes a task learned from demonstration as a Finite State Machine.

Data (end effector position and force) was recorded from the robot, as the hunam was providing kinesthetic demonstrations, by physically guiding it. The task was segmented into actions based on the change of variance observed in the demonstrations. 

For each action the arm trajectory was encoded using a CDS (Coupled Dynamical Systems) approach, see [CDS Model Learning](https://github.com/alpais/cds_model_learning_generic). Force nd stiffness modulation profiles were encoded using Gaussian Mixture Models. 
