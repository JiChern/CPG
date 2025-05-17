
# Free Gait Transition and Stable Motion Generation Using CPG-based Locomotion Control for Hexapod Robots
[click here to read the paper](https://link.springer.com/article/10.1007/s11071-024-10550-w#citeas)
# Authors
[Ji Chen](mailto:ji.chenuk@gmail.com), [Li Fan](mailto:fanli77@zju.edu.cn) and [Chao Xu](mailto:cxu@edu.zju.cn)


## Introduction
This repository includes code implementations of the paper titled "Free Gait Transition and Stable Motion Generation Using CPG-based Locomotion Control for Hexapod Robots" . The research involves an enhanced central pattern generator and a limb motion generator that considers stability criteria.

## Improved CPG model
Based on the analysis of undesired phase locking phenomena in the original diffusive CPG model[[1]](#1), we propose an enhanced model that allows for gait transition between arbitrary gaits. Additionally, the system dynamics during the transient process are improved. The system dynamics of the proposed model are governed by the following differential equations:

```math
\dot{z}_i = \begin{cases}
    F(z_i) + \gamma_i \text{Perp}_{z_i} (R(\theta_i)z_{i+1}-z_i)\text{,}  & \text{for } i=1,2,...N-1\text{,} \\ 
    F(z_i) + \gamma_i\text{Perp}_{z_i}(R(\theta_i)z_{1}-z_i)\text{,}      &\text{for } i=N\text{.}
\end{cases}
```
where $z_i$ represents the state vector of a neuron, $F(z_i)$ is the internal oscillator of the neuron, $R(\theta_i)$ is a 2D rotation matrix with the desired phase lag $\theta_i$, and $\gamma_i$ indicates the coupling strength. The function $\text{Perp}_{z_i}$ is introduced to optimize the transient dynamics. We have uploaded a MATLAB version of the model, which can be executed to observe its performance in gait transition.

<p align="center">
  <img src="https://github.com/JiChern/CPG/blob/main/fig/gait_transition_curves.png?raw=true" alt="Sublime's custom image"/>
</p>

Other types of CPG models [[2]](#1)[[3]](#1) are also available in the 'cpg_matlab' folder, which can be executed to compare with the proposed model.

## CPG-based locomotion control for hexapod robot
We propose a motion generator that is based on the proposed CPG model. This motion generator is responsible for planning the leg motion trajectories for both the stance and swing stages, while also taking into consideration the stability criteria. The overall control framework can be seen in the figure below.

<p align="center">
  <img src="https://github.com/JiChern/CPG/blob/main/fig/motion_fram.jpg?raw=true" alt="Sublime's custom image"/>
</p>

The Python implementation of the motion generation framework has been uploaded in the 'hex_motion_gen' folder. The package allows users to generate limb motion trajectories for further evaluations. It is recommended to evaluate the generated joint trajectories in the Adams simulator.


### Gait transition from caterpillar to metachronal
<p align="center">
  <img src="https://github.com/JiChern/CPG/blob/main/fig/cater_metach.gif?raw=true" alt="Sublime's custom image"/>
</p>

### Gait transition from tetrapod to caterpillar
<p align="center">
  <img src="https://github.com/JiChern/CPG/blob/main/fig/tetra_cater.gif?raw=true" alt="Sublime's custom image"/>
</p>

# Installation
Prerequisites: Ubuntu 20.04, ROS Noetic with Python 3.8, and Gazebo installed.
## create a workspace:
```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
## install the packages:
Clone the hex_motion_gen directory, copy the folders in src directory into the src folder in your workspace. After that, run
```console
$ cd ~/catkin_ws/
$ catkin_make
```
## source the workspace:
```console
$ source ~/catkin_ws/devel/setup.bash
```

# How to use the packages
## Launch the robot model and controllers in Gazebo simulator:
```console
$ roslaunch hexapod gazebo.launch
```
## Run the gait generator
```console
$ rosrun hexapod_control gait_generator.py
```
The gait_generator.py is the Python implementation of the proposed Central Pattern Generator (CPG) model. You can set the initial
gait in this script by changing the input of the function 'set_theta' defined in line 406. All available gait configurations 
are defined in lines 315 to 357.

## Run the trajectory generator
Open a new terminal and run
```console
$ rosrun hexapod_control execute_trajectory_gen.py
```
The execute_trajectory_gen.py is the Python implementation of the proposed hexapod limb motion generator. The initial gait and target
gait can be set by modifying the definitions of the 'start_gait' and 'target_gait' parameters, which are defined in lines 60 and 61. 
It is important to note that the initial gait of the trajectory generator must match the initial gait setting of the gait generator.

## Get the motor data of the robot
After observing the complete gait transition process of the robot in the Gazebo simulator, you can terminate the trajectory generator 
by pressing ctrl+C. The motion trajectories that have been saved can be located in the 'motor_data' folder. These motor data can be 
utilized in the Adams or other simulator for further evaluations.

## Reset the robot
Execute the 'respawn_model.py' node to restore the robot model in the Gazebo simulation. It is important to remember to terminate this
node after the robot has been respawned.
```console
$ rosrun hexapod_control respawn_model.py
```


## References

<a id="1">[1]</a> 
Haitao Yu and Haibo Gao and Liang Ding and Mantian Li and Zongquan Deng and Guangjun Liu (2016). 
Gait generation with smooth transition using CPG-based locomotion control for hexapod walking robot. 
IEEE Transactions on Industrial Electronics, 63, 5488-5500.

<a id="1">[2]</a> 
Ludovic Righetti and Auke Jan Ijspeert (2008). 
Pattern generators with sensory feedback for the control of quadruped locomotion. 
2008 IEEE International Conference on Robotics and Automation, 819-824.

<a id="1">[3]</a> 
Wei Xiao and Wei Wang (2014). 
Hopf oscillator-based gait transition for a quadruped robot. 
2014 IEEE International Conference on Robotics and Biomimetics, 2074-2079.


