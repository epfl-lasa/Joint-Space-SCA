## Real-Time Self-Collision Avoidance in Joint Space for Humanoid Robots

### Abstract
In this work, we propose a real-time self-collision avoidance approach for whole-body humanoid robot control. To achieve this, we learn the feasible regions of control in the humanoid's joint space as smooth self-collision boundary functions. Collision-free motions are generated online by treating the learned boundary functions as constraints in a Quadratic Program based Inverse Kinematic solver. 
As the geometrical complexity of a humanoid robot joint space grows with the number of degrees-of-freedom (DoF), learning computationally efficient and accurate boundary functions is challenging. We address this by partitioning the robot model into multiple lower-dimensional submodels. We compare performance of several state-of-the-art machine learning techniques to learn such boundary functions. Our approach is validated on the 29-DoF iCub humanoid robot, demonstrating highly accurate real-time self-collision avoidance.

### IEEE Robotics and Automation Letters Publication 
[https://ieeexplore.ieee.org/document/9345975](https://ieeexplore.ieee.org/document/9345975)


### Supplementary video for RA-L submission
<iframe width="560" height="315" src="https://www.youtube.com/embed/u3lTwFZFicY" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

### Code
Code for learning the SCA function/repeat the experiments (Python/C++):
[https://github.com/epfl-lasa/Joint-Space-SCA/](https://github.com/epfl-lasa/Joint-Space-SCA/)


### Citation
```markdown
M. Koptev, N. Figueroa and A. Billard, "Real-Time Self-Collision Avoidance in Joint Space for Humanoid Robots," in IEEE Robotics and Automation Letters, vol. 6, no. 2, pp. 1240-1247, April 2021, doi: 10.1109/LRA.2021.3057024.
```
