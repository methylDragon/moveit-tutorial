# MoveIt! Tutorial

![Brand](assets/moveit_logo.png)



## Introduction

The MoveIt! project is a ROS package that helps with motion planning for robotic manipulators. As such, it is especially useful for working with and planning for arms!

Features:

> - Motion Planning
>   - Generate high-degree of freedom trajectories through cluttered environments and avoid local minimums
> - Manipulation
>   - Analyze and interact with your environment with grasp generation
> - Inverse Kinematics
>   - Solve for joint positions for a given pose, even in over-actuated arms
> - Control
>   - Execute time-parameterized joint trajectories to low level hardware controllers through common interfaces
> - 3D Perception
>   - Connect to depth sensors and point clouds with Octomaps
> - Collision Checking
>   - Avoid obstacles using geometric primitives, meshes, or point cloud data
>
> <https://moveit.ros.org/>

This tutorial will only cover the basics for MoveIt!, up to and including the move_group interface. It won't cover the implementations of that interface, or any other deeper APIs, such as the pick and place pipeline, or time parametrisation.



## Pre-Requisites

Knowledge of the following will be presumed:

- Python
- C++
- Robot Operating System (ROS)



## Support my efforts!

 [![Yeah! Buy the DRAGON a COFFEE!](./assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)

[Or leave a tip! ヾ(°∇°*)](https://www.paypal.me/methylDragon)



## Credits

All credits and sources are listed inside the tutorials and references themselves.



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\
```