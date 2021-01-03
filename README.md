# Autonomous-Navigation-F-86-Sabre
Implementation of a Linear Quadratic Regulator (LQR) and a Model Predictive Controller (MPC) while a Recursive Rewarding Adaptive Cell Decomposition, together with Smooth 3D Path Planning, and a non-linear guidance law are put into effect.

The carpet "Functions" contains all required Matlab functions to run the pricipal script.

The carpet "Simulation" contains the principal script that must be runned "main.m". Also, there are different structs that include the reference trajectory, reference velocity profile, linearization data, control parameters and the solutions data for controllers LQR and MPC. In addition, the Simulink files to run each simulation are included, together with the file employed to conduct a simulation in FlightGear at real time. 

Results can be plotted in main.m easily, by means of loading all structs and running the sections "ANALYSIS OF BOMBS IMPACTS" and "PLOTS".
