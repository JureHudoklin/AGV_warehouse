# AGV_warehouse

This is a project I am currently working on. I am working on this also as a final project for my undergraduate degree (diploma). In this project I am developing and programing a robot ("robot platform") that could be used in simulation of a warehouse environment. In a simulation these robots would have to found their way around warehouse based on the input provided by the master computer. Each robot has to follow guiding lines, know his position based on ground markings, be able to autonomously charge itself, stop and report in case of obstacles and load cargo from designated stations. 

<image src="robot_v4/step_modeli/robot_picture.jpg">

I already completed the robot design phase. I have designed the pcb plate as well as the whole assembly. The robot will be assembled in next few weeks when I receive all the necessary parts. This part of the project is located in ./agv-skladiscni-v1/ (KiCad files) and ./Step_modeli/ (step models of robot and pcb plate).

The logic of the robot will run on a BeagleBone blue board. The board will run a ROS operating system. This is the part I have just started working on. I am writing the code in C++ which is the first bigger program I am writing in C++. I expect to learn C++ quite well when I am finished with this project. The code part of this project can be found in the ./catkin_ws/src/ folder. I have not written a lot of code yet bet the currently working part is locate in robot_control package (./catkin_ws/src/robot_control/src/...).
