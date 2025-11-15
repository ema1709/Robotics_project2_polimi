# Robotics Project 2 – Politecnico di Milano  
Mapping, Simulation, and Autonomous Navigation with ROS

This repository contains the implementation of the **second robotics project** for the Robotics course at Politecnico di Milano.  
The objective is to reconstruct a 2D map of an indoor environment from real robot data, set up a simulation of the robot using Stage, and configure a full **ROS Navigation Stack** capable of autonomously reaching a sequence of goals read from a CSV file.

The project is developed in **C++** using **ROS Noetic**.

---

## 🧭 Project Overview

The project is divided into two main tasks:

### **🔹 Task 1 — Mapping**
Using the provided ROS bag:
- Convert `/odometry` into a proper TF (`odom → base_link`)
- Merge the two laser scans (`/scan_front` + `/scan_back`) into a full 360° scan
- Filter out robot-body points before feeding the merged scan to the mapper
- Use a mapping package (GMapping / Hector / Cartographer / SLAM Toolbox)
- Provide a launch file that:
  - runs conversion nodes
  - starts the SLAM node
  - opens RViz with `map` as global frame

The output is a **2D occupancy map** (`.png + .yaml`).

---

### **🔹 Task 2 — Navigation**
Using the generated map:
- Create a robot simulation in **Stage**
- Choose kinematics (Ackermann / skid-steering / omnidirectional)
- Configure the **ROS Navigation Stack** using the map
- Implement a **goal publisher**:
  - read goals from `csv/goals.csv`
  - send goals via ROS Actions (`move_base`)
  - send next goal only after success/abort
- Provide a launch file that:
  - starts Stage with the environment
  - loads the navigation stack
  - starts the goal controller
  - opens RViz (map, TFs, scan, AMCL particles, paths, goals)

---

## 🗺️ Generated Map

Below is the reconstructed map obtained from Task 1:

![map](map.png)

*(File stored in `second_project/map/map.png`)*

---
