
# Free Gait Transition and Stable Motion Generation Using CPG-based Locomotion Control for Hexapod Robots
## Introduction
This repository includes code implementations of the paper titled "Free Gait Transition and Stable Motion Generation Using CPG-based Locomotion Control for Hexapod Robots". The research involves an enhanced central pattern generator and a limb motion generator that considers stability criteria

## Improved CPG model
Based on the analysis of undesired phase locking phenomena in the original diffusive CPG model[[1]](#1), we propose an enhanced model that allows for gait transition between arbitrary gaits. Additionally, the system dynamics during the transient process are improved. The system dynamics of the proposed model are governed by the following differential equations:

```math
\dot{z}_i = \begin{cases}
    F(z_i) + \gamma_i \text{Perp}_{z_i} (R(\theta_i)z_{i+1}-z_i)\text{,}  & \text{for $i=1,2,...N-1$}\text{,} \\ 
    F(z_i) + \gamma_i\text{Perp}_{z_i}(R(\theta_i)z_{1}-z_i)\text{,}      &\text{for $i=N$}\text{.}
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


