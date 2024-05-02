# ADS Project - Truck Platooning using CARLA Simulator



# Introduction

In this project, you will build an autonomous truck platooning system based on techniques developed by the previous ADS projects (i.e., lane detection and object detection). The CARLA autonomous driving simulator is used with a slight modification to support semi-trailer trucks. You can deploy trucks as many as you want, and each truck is equipped with a front camera and a front radar. The camera can be utilized to detect lanes and drivable areas, while the radar can detect preceding vehicles. Also, each truck can be controlled by accelerating and braking as well as steering. The sensors and actuators can be reached through a ROS 2 network.

The objective of this project is to develop a truck platooning system with at least three trucks that can maintain a constant gap between them while maintaining the platoon within a lane.


# Background Information

You can use the CARLA simulator and ROS 2 bridge software at https://github.com/AveesLab/carla-virtual-platoon. The repository contains information to deploy your own CARLA simulation environment with trucks that can be controlled by ROS 2. The detailed description for the installation and test can be found at the repository.


# Project Goals & Objectives

The goals and objectives of the Truck Platooning using CARLA Simulator project are as follows:

1. To deploy the CARLA simulator on your PC and test the interface for receiving sensor data and controlling the trucks.
2. To develop the lateral control algorithm, which can be developed by either following the lane or following the preceding truck.
3. To develop the longitudinal control algorithm, where its primary objective is to maintain a constant gap between the trucks. You have to develop an external interface that provides the platoon's desired speed and the gap distance to the leading vehicle (LV).
4. To develop V2V communication between trucks, where relevant information (e.g., disired gap distance) should be delivered to neighbering trucks. If you don't have real V2V hardware, you can simply use ROS 2 network between trucks for the emulation of V2V networks.

5. # Project Timeline

The following is a sample project timeline:

1. Week 1: Study about the truck platooning concept and analyze the technical requirements of the project.
2. Week 2: Install the CARLA simulator can test the basic interface that can be found at https://github.com/AveesLab/carla-virtual-platoon.
3. Week 3: Integrate your lane detection algorithm and optimize it using a manually driven single truck.
4. Week 4: Develop a lateral control algorithm based on the lane detection results.
5. Week 5: Study the string stability issue of platooning and the radar interface provided by the CARLA simulator.
6. Week 6: Develop a basic longitudinal control algorithm using two trucks.
7. Week 7-8: Optimize the control performance by parameter tunings and advanced control algorithms.
8. Week 9-10: Develop a V2V communication and upgrade the control algorithm based on V2V communications.
9. Week 11-12: Final preparation and submission. Participants should use this time to finalize their project reports, document their code, and prepare their submissions. The final project submissions are due at the end of week 12.

Shield: [![CC BY-NC-SA 4.0][cc-by-nc-sa-shield]][cc-by-nc-sa]

This work is licensed under a
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License][cc-by-nc-sa].

[![CC BY-NC-SA 4.0][cc-by-nc-sa-image]][cc-by-nc-sa]

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg
