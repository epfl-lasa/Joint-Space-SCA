## Real-Time Self-Collision Avoidance in Joint Space for Humanoid Robots

### Abstract
```markdown
In this work, we propose a real-time self-collision avoidance approach for whole-body humanoid robot control. To achieve this, we learn the feasible regions of control in the humanoid's joint space as smooth self-collision boundary functions. Collision-free motions are generated online by treating the learned boundary functions as constraints in a Quadratic Program based Inverse Kinematic solver. 
As the geometrical complexity of a humanoid robot joint space grows with the number of degrees-of-freedom (DoF), learning computationally efficient and accurate boundary functions is challenging. We address this by partitioning the robot model into multiple lower-dimensional submodels. We compare performance of several state-of-the-art machine learning techniques to learn such boundary functions. Our approach is validated on the 29-DoF iCub humanoid robot, demonstrating highly accurate real-time self-collision avoidance.
```
### Supplementary video for RA-L submission
[Link](https://www.youtube.com/watch?v=u3lTwFZFicY)
<iframe width="560" height="315" src="https://www.youtube.com/embed/u3lTwFZFicY" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

### Code
Code for learning the SC function/repeat teh experiments in simulation/on real robot (Python/C++):
[Link](https://github.com/epfl-lasa/Joint-Space-SCA/)

### RA-L Paper 
[Link](https://ieeexplore.ieee.org/document/9345975)

**Bold** and _Italic_ and `Code` text

[Link](url) and ![Image](src)

