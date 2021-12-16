# AER1216 - Development of Fixed-Wing and Multi-Rotor sUAS

## Group Members
Farhan Wadia

Shahzeb Mahmood

David Rolko

## Fixed-Wing sUAS Development
In the [Part 1 - Fixed-Wing](Part%201%20-%20Fixed-Wing) folder, open and run the `Fixed_Wing_Range_Endurance.m` file to view the analysis and plots for range and endurance.

For the other parts of the analysis and simulation, first open and run the `FW_control_design.mlx` file. 

After doing that, open the `Fixed_Wing_Aircraft_Sim_ssv2.slx` file in Simulink and run it for the simulation. 

## Multi-Rotor Drone Development
In the [Part 2 - Multi-Rotor](Part%202%20-%20Multi-Rotor) folder, the range and endurance calculations can be found in the `Part2_Q1.mlx` file.

All other analysis for the multi-rotor (quadrotor dynamics, position estimation, position control, simulation model) can be found in the `Part2_SimulinkModel.slx` file. This Simulink model relies on the reference input signals saved in the `input_signals.mat` file. 

The system response for this input has been saved in the `position_responses.mat` file, and the `multi_rotor_response_plot.m` script plots this data for use in the presentation.
