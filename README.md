# Slicer_ROS_Extension
To support the research and prototyping of medical robots, we have proposed a ROS module for 3D Slicer. 
This will allow users to connect 3D Slicer's image guidance capabilities with ROS's robotics capabilities, enabling real-time visualization of robots and connections to dynamics simulators like AMBF, or other ROS tools like gazebo or RViz.

## Overview
The project consist of two parts: The 3D Slicer ROS Module and the AMBF simulation of Galen Surgical Robot Plugin for 3D Slicer. 
The 3D Slicer ROS Module handles the information and communication via ROS. The AMBF simulator Plugin is capable of publishing robot states and robot visualization materials to ROS for the 3D Slicer Module to accquire.

## 1.Installation

### 1.1 Install ROS Noetic
[See ROS Wiki Link](http://wiki.ros.org/noetic/Installation/Ubuntu)

### 1.2 Install and build AMBF2.0
Clone and build AMBF2.0  
```bash
git clone https://github.com/WPI-AIM/ambf.git
cd ambf
git checkout -b ambf-2.0 origin/ambf-2.0
git pull
```
Build and source ambf (make sure you're on branch ambf-2.0 before building) as per the instructions on [AMBFs wiki](https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF.)

### 1.3 Install and build 3D Slicer
See build instructions [Here](https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/index.html)

### 1.4 Clone and build Slicer_ROS_Extension with AMBF
```bash
git clone https://github.com/LCSR-CIIS/Slicer_ROS_Extension.git
cd 
```

---

If you find this repository useful, please cite:

```bibtex
@article{sahu2024entri,
  title={Enhanced Navigational Toolkit for Robotic Interventions},
  author={Sahu, Manish and Ishida, Hisashi and Connolly, Laura and Fan, Hongyi and Deguet, Anton and Kazanzides, Peter and Creighton, Francis X and Taylor, Russell H and Munawar, Adnan},
  journal={arXiv preprint arXiv:2401.11715},
  year={2024}
}
```
    
---
