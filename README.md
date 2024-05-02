# ADS Project - Truck Platooning using CARLA Simulator



# Introduction

In this project, you will build an autonomous truck platooning system based on techniques developed by the previous ADS projects (i.e., lane detection and object detection). The CARLA autonomous driving simulator is used with a slight modification to support semi-trailer trucks. You can deploy trucks as many as you want, and each truck is equipped with a front camera and a front radar. The camera can be utilized to detect lanes and drivable areas, while the radar can detect preceding vehicles. Also, each truck can be controlled by accelerating and braking as well as steering. The sensors and actuators can be reached through a ROS 2 network.

The objective of this project is to develop a truck platooning system with at least three trucks that can maintain a constant gap between them while maintaining the platoon within a lane.
</br>

![ezgif-7-2773c58853](https://github.com/SEA-ME/ADS_Truck-Platooning/assets/700359/c7139d48-6a11-4d45-b5db-3c6fce4ec8b1)


# Background Information




You can use the CARLA simulator and ROS 2 bridge software at https://github.com/AveesLab/carla-virtual-platoon. The repository contains information to deploy your own CARLA simulation environment with trucks that can be controlled by ROS 2. The detailed description for the installation and test can be found at the repository.
</br>

# Project Goals & Objectives

The goals and objectives of the Truck Platooning using CARLA Simulator project are as follows:

1. To deploy the CARLA simulator on your PC and test the interface for receiving sensor data and controlling the trucks.
2. To develop the lateral control algorithm, which can be developed by either following the lane or following the preceding truck.
3. To develop the longitudinal control algorithm, where its primary objective is to maintain a constant gap between the trucks. You have to develop an external interface that provides the platoon's desired speed and the gap distance to the leading vehicle (LV).
4. To develop V2V communication between trucks, where relevant information (e.g., disired gap distance) should be delivered to neighbering trucks. If you don't have real V2V hardware, you can simply use ROS 2 network between trucks for the emulation of V2V networks.
</br>

# Project Timeline

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
</br>

# Submission

Upon completion of the project, participants should submit a GitHub repository that includes the following components:

1. Software Code: Participants should provide the source code for their truck platooning software, including the perception, decision, and control modules. The code should be well-documented, with clear explanations of the algorithms and techniques used.
2. Simulation Results: Participants should provide simulation results that demonstrate the performance of their trucks in the simulation environment. This may include screenshots or videos in action, as well as metrics and statistics that measure the performance.
3. Project Report: Participants should provide a project report that describes their experience with the project, including any challenges faced and how they were overcome. The report should also include a discussion of the algorithms and techniques used, as well as the results and conclusions of the project.

By providing these components, the participants will demonstrate their understanding of the concepts and techniques involved in the project, and will provide evidence of their ability to implement an autonomous system. The GitHub repository will serve as a portfolio of the participants' work, and will provide a record of their achievements and contributions to the field of autonomous systems.  
</br>

# References

Here are some open-source references that could be used for the truck platooning project:

1. ROS (Robot Operating System) (http://www.ros.org/): ROS is an open-source robotic operating system that provides a wide range of tools and libraries for developing autonomous systems, including robots and drones. It can be used to build the software framework for the Mail Delivering PiRacer and manage its various components, such as the navigation and delivery modules.
2. Gazebo (http://gazebosim.org/): Gazebo is an open-source robotics simulator that provides a realistic environment for testing and evaluating autonomous systems. Participants can use Gazebo to test and refine their Mail Delivering PiRacer in a virtual environment, before deploying it in the real world.
3. OpenCV (https://opencv.org/): OpenCV is an open-source computer vision library that provides a wide range of algorithms and functions for image and video processing. It can be used to implement various computer vision techniques, such as object detection and tracking, for the Mail Delivering PiRacer.
4. CARLA (https://carla.org): CARLA is an open-source autonomous driving simulator based on the Unreal Engine. It provides a realistic 3D driving environment with various vehicles. Since it is convinient to make an interface between CARLA and ROS. We can easily make and test autonomous driving algorithms based on the CARLA simulator.

Shield: [![CC BY-NC-SA 4.0][cc-by-nc-sa-shield]][cc-by-nc-sa]

This work is licensed under a
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License][cc-by-nc-sa].

[![CC BY-NC-SA 4.0][cc-by-nc-sa-image]][cc-by-nc-sa]

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg
