# Intro to Control of Mobile Robotics (USF)

This repository is an extension of the project framework, [FAIRIS-Lite](https://github.com/biorobaw/FAIRIS-Lite), enabling users to implement navigational control logic for robots in the Webots simulation. It includes the necessary Python controller files that I have created for the various lab tasks required by this course.

The controller files are used to simulate robot motion and sensor readings in the Webots development environment. The free, open source Webots simulator can be found [here](https://cyberbotics.com/).

The controller files are the main deliverables for each lab and can be found in the select lab folders.

Each Lab will be explained in more detail in the following sections:

---

## Lab 1 - Kinematics

### Objective
The objective of this lab is to learn about motion control and kinematics for a robot to navigate through a set of waypoints.

<p align="center">
  <em>Intended robot path for Maze 1</em>
<img width="931" height="711" alt="Robot path" src="https://github.com/user-attachments/assets/531bc600-3776-4154-93a6-7d4f0489ae68" />
</p>
<p align="center">
</p>

To accomplish this task, several functions were implemented to perform straight-line, curved, and rotational motions.  
A full report with more details and calculations for this lab can be found in the `Lab_1` folder.  

### Code
[Lab 1 Controller](lab1/lab1.py)

### Demo Video
[▶ Watch Lab 1 Demo](https://1drv.ms/v/c/0456e9a6af993fe4/ERfQHhCjz7dFjYCLxt1J7HwBrIa7K6sf6w7CPYwhzb9_ug?e=WE0yMk)

---

## Lab 2 - PID and Wall Following

### Objective
The objective of this lab was to apply a PID controller to navigate parallel to a wall and stop at adesired distance from an end wall. The lab will also focus on how Lidars are used to measure distances to walls.

<p align="center">
<em>LiDar Reading Sensors</em>
</p>
<p align="center">
<img width="390" height="254" alt="image" src="https://github.com/user-attachments/assets/193716ea-564e-4cf1-97bc-6b577d53cb51" />
</p>

The PID controller will use the LIDAR sensor to control robot navigation. It will use Kp, Ki, Kd PID “forward” gain constants for motion control, applying them only to the error values related to the forward motion. The control should be applied exclusively towards the robot’s front motion, stopping 1m away from the end wall, and should not influence side motions.

<p align="center">
<em>Flowchart displaying proportional gain process</em>
</p>
<p align="center">
<img width="677" height="119" alt="image" src="https://github.com/user-attachments/assets/314c785b-eddf-419d-9c3d-fbcf9bfdfcb1" />
</p>

### Code
- [Lab 2 Task 1 Controller](lab2/lab2_task1.py)  
- [Lab 2 Task 2 Controller](lab2/lab2_task2.py)

### Demo Videos
- [▶ Watch Lab 2 Task 1](https://1drv.ms/v/c/0456e9a6af993fe4/Eag3bxR8B_5HnEFshaAkeqgBzwUni0caFjEfL3mv09gJAg?e=QTzNwd)  
- [▶ Watch Lab 2 Task 2](https://1drv.ms/v/c/0456e9a6af993fe4/EUMrSz_LkIFPhZBZT7pA0jkBoMwUrO8uCsQMSOiG2uxPTA?e=CJ7MiQ)  

---

## Lab 3 - Motion to Goal and Bug 0

### Objective
The objective for this lab is to plan motion fora robot to reach a goal while avoiding obstacles.
It will include how to utilize a camera with object detection and the bug zero algorithm to navigate through an obstacle-rich environment to reach a goal location

<p align="center">
<em>Robot Point of View</em>
</p>
<p align="center">
<img width="809" height="288" alt="image" src="https://github.com/user-attachments/assets/513dccbb-3400-47d7-9b0a-95c7d0f6d945" />
</p>
<br>
<br>
<br>

These will be the mazes for this lab where the robot will navigate the maze according to the specific color of the obstacle it intends to reach.
<p align="center">
<img width="809" height="288" alt="image" src="https://github.com/user-attachments/assets/813db91a-cd80-4860-ab21-7b647d0ef6d2" />
</p>
<p align="center">
<em>Maze for task 1</em>
</p>

<p align="center">
<img width="558" height="419" alt="image" src="https://github.com/user-attachments/assets/eec39599-0740-4cea-9ffa-8cdad11b5835" />
</p>
<p align="center">
<em>Maze for task 2</em>
</p>

### Code
- [Lab 3 Task 1 Controller](lab3/lab3_task1.py)  
- [Lab 3 Task 2 Controller](lab3/lab3_task2.py)

### Demo Videos
- [▶ Watch Lab 3 Task 1](https://1drv.ms/v/c/0456e9a6af993fe4/Ef9scghueQhOmT8u23jbK2UBWTnxGhoiI917TxhSTT1RBA?e=XhlDZl)  
- [▶ Watch Lab 3 Task 2](https://1drv.ms/v/c/0456e9a6af993fe4/EUij5seCICZHmber0oaAMl0Bt4HsZ1Box1_mlZx08v9scA?e=SFHMYZ)  

---

## Lab 4 - Localization

### Objective
The objective of this lab was to utilize probabilistic robot localization. Enoders, Compass readings, LiDAR sensors and cameras with object recognition were incorporated into this solution.

<p align="center">
<em>RGB Values for Maze Obstacles</em>
</p>
<p align="center">
<img width="662" height="440" alt="image" src="https://github.com/user-attachments/assets/449690d8-9907-4b5d-b253-3a31c45ba993" />
</p>

<p align="center">
<em>Wall Configuration Using Cardinal Points System</em>
</p>
<p align="center">
<img width="908" height="624" alt="image" src="https://github.com/user-attachments/assets/c9662d32-8294-4d2c-ae10-3584170dbd40" />
</p>


### Code
- [Lab 4 Task 1 Controller](lab4/lab4_task1.py) 
- [Lab 4 Task 2 Controller](lab4/lab4_task2.py)

### Demo Videos
- [▶ Watch Lab 4 Task 1](https://1drv.ms/v/c/0456e9a6af993fe4/Ef4JC1a1YolIo57hhy0jRKoB46xyzG7l5BpEy16AlvTW7A?e=bkbUfa)  
- [▶ Watch Lab 4 Task 2](https://1drv.ms/v/c/0456e9a6af993fe4/EV4LBfI-GgdPsf6TDzAYdeoBOaxyI3M3wj0v5bWLpxAUWQ?e=tPHcId)  
