
# Free Gait Transition and Stable Motion Generation Using CPG-based Locomotion Control for Hexapod Robots
## Introduction
This repository includes code implementations of the paper titled "Free Gait Transition and Stable Motion Generation Using CPG-based Locomotion Control for Hexapod Robots". The research involves an enhanced central pattern generator and a limb motion generator that considers stability criteria

## Improved CPG model
Based on the analysis of undesired phase locking phenomena in the original diffusive CPG model, we propose an enhanced model that allows for gait transition between arbitrary gaits. Additionally, the system dynamics during the transient process are improved. The system dynamics of the proposed model are governed by the following differential equations:

```math
\dot{z}_i = \begin{cases}
    F(z_i) + \gamma_i \text{Perp}_{z_i} (R(\theta_i)z_{i+1}-z_i)\text{,}  & \text{for $i=1,2,...N-1$}\text{,} \\ 
    F(z_i) + \gamma_i\text{Perp}_{z_i}(R(\theta_i)z_{1}-z_i)\text{,}      &\text{for $i=N$}\text{.}
\end{cases}
```
where $z_i$ represents the state vector of a neuron, $F(z_i)$ is the internal oscillator of the neuron, $R(\theta_i)$ is a 2D rotation matrix with the desired phase lag $\theta_i$, and $\gamma_i$ indicates the coupling strength. The function $\text{Perp}_{z_i}$ is introduced to optimize the transient dynamics. We have uploaded a MATLAB version of the model, which can be executed to observe its performance in gait transition.
