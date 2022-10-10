## What is
This is the repository which contains the workspace of the second project of the course **Robotics** at Politecnico di Milano.

University: **Politecnico di Milano, Laurea Magistrale (MSc) - Computer Science and Engineering**

## The project
We had to develop this project with ROS, and we had some bags recorded.
The robot with which were recorded, was an omnidirectional robot with mecanum wheels (4 wheels with rollers at 45°); is the same robot of the first project. In the bag files, the following topics were published:
- `/odom`: odometry topic;
- `/front/scan`: laser mounted in front of the robot;
- `/rear/scan`: laser mounted on the rear of the robot;
- `/wheel_states`: data from wheels (with the same structure as the first robotics project: for further information see the relative repository);
- `tf_static`: static tf for laser position.

Note that the front scan and the rear scan were different.
We had three bag files: one had to be used for map creation, and two for localization (we were allowed to choose the one we preferred).

The goal was to:
1. write launch files to create the map;
2. write the launch file to perform amcl based localization;
3. write a service to save an image with the map and the trajectory of the robot.

We were allowed to choose the package for map creation, but we had to use amcl for localization.
A node was needed to publish the odometry as a tf, and a node was needed to merge the two lasers.

A launch file for the mapping and a launch file for amcl localization were required. The professor required to be able to create a map and start amcl with the launch files, hence we needed to include everything in there. The professor required to be able to:
- start the bag file in a new terminal;
- start the map server to save the image (for the mapping task);
- call the service to save the image with both the map and the trajectory on it.

## Our Team
- Luca Alessanrini  
- Massimo Buono (added as collaborator)
- Mattia Portanti (added as collaborator)

## How we divided the work
We decided to face this challenge for the most part with on-line meetings, where we discussed and implemented together different choices

## How to replicate our results
We developed this on Ubuntu 18, with ROS Melodic. Once having downloaded the workspace, it is necessary to link it into the bashrc file and build it in order to use the project.

## Submission
We had to submit not the whole workspace, but just only our package.
