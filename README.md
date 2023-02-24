# Mobile Manipulation Project with TIAGo robot and ROS
### NOTE: This project was made in a course at Royal Institute of Technology (KTH). Due to university's policies regarding plagiarism, no copies of codes are allowed from this repository before July 2024. The authors will not take any responsibilities for plagiarism made by students at KTH or by someone at other universities/institutions. This repository is uploaded for personal purposes. 

This was a final project in a robotics course at Royal Institute of Technology (KTH). Working modules for path planning, navigation, sensing and manipulation with minimal documentation
were provided for this project. The modules included pre-made messages and services.
The task given to us was to design a state machine and behaviour tree to fulfil a misson of picking a cube from one table and place it on a another table.
The scripts were written in Python to be able to work with ROS (Robot Operating System) and simulate the mission in Gazebo and RViz. The state machine and the behaviour 
tree were required to do following things:

## State machine
* Picking up a cube without sensors by using a known pose
* Move to the table behind
* Place the cube on the table blindly by a known pose.

Following is a video showing the robot doing the task:

https://user-images.githubusercontent.com/62840946/221251757-fd95cffc-dae7-42fd-836d-750573895327.mov


## Behaviour tree
* The robot should localise itself
* Plan a path to the cube and navigate to the table with the cube
* Detect the cube with its camera and pick up the cube
* Navigate to the a table in another room
* Place the cube on the table and detect is the cube has been placed correctly
* If the robot is kidnapped during navigation, it should be able to detect it and relocate itself before replanning its path.
* If the cube is not correctly placed, the cube should respawn to the first table and the robot should redo the mission.

Following video shows the robot localising itself and start its navigation to the first table to pick up the cube:

https://user-images.githubusercontent.com/62840946/221252129-b96730d2-4b32-4cee-a0b4-a1674f8af2a1.mov

## Following codes existing in this repository
[sm_students.py](https://github.com/tawsiislam/Mobile_Manipulation_Project/blob/main/sm_students.py) contains the Python scripts for the state machine.

[bt_students.py](https://github.com/tawsiislam/Mobile_Manipulation_Project/blob/main/bt_students.py) contains the main scripts for building the behaviour tree.

[behaviours_student.py](https://github.com/tawsiislam/Mobile_Manipulation_Project/blob/main/behaviours_student.py) contains the implementations for localising, handling 
kidnapping, navigation, and picking & placing by using ROS messages and services.
