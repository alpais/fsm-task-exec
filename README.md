## fsm-task-exec

Legacy code: addon module for [RobotToolKit](https://github.com/epfl-lasa/robot-toolkit). This package executes a task learned from demonstration as a Finite State Machine.

End effector position and force were recorded from the robot, as the human was providing kinesthetic demonstrations, by physically guiding it. The task was segmented into actions based on the change of variance observed in the demonstrations. 

For each action the arm trajectory was encoded using a CDS (Coupled Dynamical Systems) approach, see [CDS Model Learning](https://github.com/alpais/cds_model_learning_generic). Force and stiffness modulation profiles were encoded using Gaussian Mixture Models. 
