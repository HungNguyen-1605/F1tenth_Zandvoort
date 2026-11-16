# F1TENTH Autonomous Racing

\<p align="center">
&#x20; \<strong>LiDAR-Based Autonomous Racing Using Follow-the-Gap and Disparity Extender\</strong>
\</p>

\<p align="center">
&#x20; A ROS 2 autonomous driving project developed and tested in the F1TENTH Gym simulation environment.
\</p>

\<p align="center">
&#x20; \<img src="[https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros)">HungNguyen-1605
&#x20; \<img src="[https://img.shields.io/badge/Ubuntu-22.04-orange?logo=ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?logo=ubuntu)">
&#x20; \<img src="[https://img.shields.io/badge/Python-3-green?logo=python](https://img.shields.io/badge/Python-3-green?logo=python)">
&#x20; \<img src="[https://img.shields.io/badge/Platform-F1TENTH-red](https://img.shields.io/badge/Platform-F1TENTH-red)">
\</p>

---

## Table of Contents

- [Overview](#overview)
- [Project Objectives](#project-objectives)
- [Simulation Environment](#simulation-environment)
- [System Architecture](#system-architecture)
- [Autonomous Driving Algorithm](#autonomous-driving-algorithm)
- [ROS 2 Communication](#ros-2-communication)
- [Repository Structure](#repository-structure)
- [Available Maps](#available-maps)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [Changing the Track](#changing-the-track)
- [Configuration](#configuration)
- [Testing and Evaluation](#testing-and-evaluation)
- [Experimental Results](#experimental-results)
- [Limitations and Future Improvements](#limitations-and-future-improvements)
- [Acknowledgements](#acknowledgements)

---

## Overview

This project focuses on developing a LiDAR-based autonomous driving controller for the **F1TENTH platform** using ROS 2.

The autonomous vehicle operates in the F1TENTH Gym simulation environment and uses real-time LiDAR measurements to identify obstacles, detect available driving space, and generate steering and speed commands.

The main objective is to develop a controller capable of navigating a racing track autonomously while maintaining safe distances from track boundaries and obstacles.

The project focuses on:

- LiDAR-based perception.
- Follow-the-Gap (FTG) obstacle avoidance.
- Disparity Extender for improving obstacle clearance.
- Steering and speed control.
- Autonomous racing performance evaluation.

### Simulation Preview

<img width="1920" height="1080" alt="SIMULATION SCREENSHOT SHOWING THE F1TENTH VEHICLE ON THE TRACK" src="https://github.com/user-attachments/assets/234e2030-dbd7-4856-a9f2-6318232c571a" />
---

## Project Objectives

The primary objective is to develop and evaluate an autonomous racing controller in a simulated F1TENTH environment.

The project focuses on the following goals:

### Collision-Free Lap Completion

Develop a controller capable of completing consecutive laps without colliding with track walls or obstacles.

### Lap Time Optimization

Improve racing performance by reducing lap completion time while maintaining stable and safe driving behavior.

### Driving Stability

Evaluate the controller's ability to navigate different track sections, including straight roads, corners, and narrow passages.

---

## Simulation Environment

The project uses the F1TENTH Gym ROS simulation environment.

The simulator provides a virtual racing vehicle, track maps, LiDAR sensor data, and vehicle control interfaces through ROS 2 topics.

### Development Environment

| Component            | Configuration   |
| -------------------- | --------------- |
| Operating System     | Ubuntu          |
| ROS Distribution     | ROS 2 Humble    |
| Programming Language | Python          |
| Simulation Platform  | F1TENTH Gym ROS |
| Perception Sensor    | 2D LiDAR        |
| Vehicle Model        | F1TENTH         |
| Main Test Track      | Zandvoort       |

### Simulation Screenshot

<img width="1604" height="906" alt="SCREENSHOT OF THE F1TENTH SIMULATION ENVIRONMENT" src="https://github.com/user-attachments/assets/e2a99a15-ea3c-424d-acb2-d32c63d24787" />
---

## System Architecture

The system consists of two main components:

- **F1TENTH Gym ROS Simulator**
- **LiDAR-Based Autonomous Controller**

The simulator publishes LiDAR and odometry information. The autonomous controller processes these inputs and publishes driving commands to control the ego vehicle.

### ROS 2 Architecture

```text
             F1TENTH Gym ROS Simulator
                         |
             +-----------+-----------+
             |                       |
             v                       v
          /scan              /ego_racecar/odom
       LaserScan                  Odometry
             |                       |
             +-----------+-----------+
                         |
                         v
              /disparity_controller
                         |
               LiDAR Processing
                         |
                Gap Identification
                         |
                Steering Control
                         |
                  Speed Control
                         |
                         v
                      /drive
             AckermannDriveStamped
                         |
                         v
              F1TENTH Ego Vehicle
                         |
                         v
                    Simulation
```

### Main Components

| Component            | Description                                                                      |
| -------------------- | -------------------------------------------------------------------------------- |
| Simulator            | Provides the virtual racing environment and vehicle model.                       |
| LiDAR Subscriber     | Receives real-time LiDAR scan measurements.                                      |
| Odometry Subscriber  | Receives vehicle position and motion information.                                |
| Disparity Processing | Identifies significant distance differences between adjacent LiDAR measurements. |
| Gap Selection        | Identifies available driving space.                                              |
| Steering Controller  | Generates steering commands based on the selected driving direction.             |
| Speed Controller     | Determines vehicle speed according to driving conditions.                        |
| Drive Publisher      | Sends steering and speed commands to the simulator.                              |

---

## Autonomous Driving Algorithm

The controller is based on LiDAR-based obstacle avoidance, combining the **Follow-the-Gap** approach with **Disparity Extender** processing.

### LiDAR Data Acquisition

The controller subscribes to the `/scan` topic to receive LaserScan measurements from the simulated vehicle.

Each measurement represents the distance between the LiDAR sensor and an object in a particular direction.

The controller processes these measurements to identify nearby obstacles and available driving space.

### Disparity Extender

The Disparity Extender technique identifies significant differences between adjacent LiDAR measurements.

A large difference in distance may indicate an obstacle boundary.

The algorithm extends or modifies the obstacle region to account for the vehicle's physical width and improve the safety margin when selecting a driving path.

### Follow-the-Gap

After processing obstacle regions, the controller searches for available open space.

The general procedure is:

1. Process the LiDAR measurements.
2. Identify obstacle regions.
3. Determine available gaps.
4. Select a suitable gap.
5. Calculate the target direction.
6. Generate a steering command.

The selected gap provides a direction that allows the vehicle to avoid detected obstacles while continuing along the track.

### Steering Control

The steering controller converts the selected target direction into a steering angle.

Steering commands are published through the `/drive` topic using the `AckermannDriveStamped` message type.

### Speed Control

Vehicle speed is adjusted according to driving conditions.

The objective is to balance driving performance and safety, especially when approaching obstacles or navigating turns.

<img width="849" height="786" alt="Screenshot from 2026-11-16 23-18-42" src="https://github.com/user-attachments/assets/9485ded5-81b0-4f5c-8dc5-899e68d0ee15" />
---

## ROS 2 Communication

The autonomous controller communicates with the F1TENTH simulator through ROS 2 topics.

### Subscribed Topics

| Topic               | Message Type                | Purpose                        |
| ------------------- | --------------------------- | ------------------------------ |
| `/scan`             | `sensor_msgs/msg/LaserScan` | Receives LiDAR measurements.   |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry`     | Receives ego vehicle odometry. |

### Published Topics

| Topic    | Message Type                               | Purpose                                               |
| -------- | ------------------------------------------ | ----------------------------------------------------- |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Sends steering and speed commands to the ego vehicle. |

### Controller Node

The controller appears in the ROS 2 node list as:

```text
/disparity_controller
```

### Communication Verification

The following communication has been verified in the running simulation:

- The simulator publishes LiDAR data through `/scan`.
- The controller subscribes to `/scan`.
- The simulator publishes vehicle odometry through `/ego_racecar/odom`.
- The controller subscribes to odometry.
- The controller publishes driving commands through `/drive`.

<img width="892" height="776" alt="Screenshot from 2026-11-16 23-21-57" src="https://github.com/user-attachments/assets/20203f05-280c-4760-8695-8cbb0716c5a2" />
---

## Repository Structure

The recommended repository organization is:

```text
F1TENTH-Autonomous-Racing/
│
├── README.md
│
├── src/
│   └── lidar_test/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │
│       ├── lidar_test/
│       │   ├── __init__.py
│       │   ├── autonomous_controller.py
│       │   ├── autonomous_drive.py
│       │   └── lidar_test.py
│       │
│       ├── autonomous_controller_v5.py
│       ├── generate_raceline.py
│       └── zandvoort_raceline.csv
│
├── config/
│   └── sim.yaml
│
├── maps/
│   ├── Mapas-F1TENTH.zip
│   ├── Mapas-F1TENTH-v2.zip
│   └── README.md
│
├── screenshots/
│
├── LICENSE
│
└── .gitignore
```

### Important Files

| File                          | Description                                                                           |
| ----------------------------- | ------------------------------------------------------------------------------------- |
| `autonomous_controller.py`    | Main autonomous controller registered as the `autonomous_drive` ROS 2 executable.     |
| `autonomous_drive.py`         | Additional driving controller implementation.                                         |
| `lidar_test.py`               | LiDAR testing node.                                                                   |
| `autonomous_controller_v5.py` | Separate controller version containing raceline and Pure Pursuit-related development. |
| `generate_raceline.py`        | Script for generating a racing line.                                                  |
| `zandvoort_raceline.csv`      | Racing line waypoint data.                                                            |
| `sim.yaml`                    | Simulator configuration, including map selection and vehicle settings.                |

The main executable is configured through `setup.py`:

```python
autonomous_drive = lidar_test.autonomous_controller:main
```

Therefore, the following command runs `autonomous_controller.py`:

```bash
ros2 run lidar_test autonomous_drive
```

---

## Available Maps

This repository supports multiple F1TENTH racing tracks.

Two map archives are included so users can extract and select the track they want to use.

### Map Archives

| Archive                | Description                       |
| ---------------------- | --------------------------------- |
| `Mapas-F1TENTH.zip`    | First collection of F1TENTH maps. |
| `Mapas-F1TENTH-v2.zip` | Additional map collection.        |

The available map collections include:

- Brands Hatch
- Budapest
- Oschersleben
- Sao Paulo
- Catalunya
- Sakhir
- Sepang
- Zandvoort

Each usable map should have its corresponding image (`.png`) and configuration (`.yaml`) files.

### Map Preview

<img width="800" height="800" alt="Zandvoort_map" src="https://github.com/user-attachments/assets/d6b79164-7379-48b9-ad6f-79570b07a9eb" /> <img width="524" height="230" alt="Screenshot from 2026-11-16 23-28-12" src="https://github.com/user-attachments/assets/3c118024-3e8f-4d38-b884-ba8753d09fec" />
---

## Installation

### Requirements

The project requires:

- Ubuntu
- ROS 2 Humble
- Python 3
- F1TENTH Gym ROS simulator
- Required ROS 2 message packages and dependencies

### Create a ROS 2 Workspace

If you do not already have a workspace:

```bash
mkdir -p ~/F1Tenth-Repository/src
cd ~/F1Tenth-Repository
```

### Clone the Repository

```bash
cd ~/F1Tenth-Repository
git clone git@github.com:HungNguyen-1605/ftg_zandvoort.git
```

If you already have the repository, simply enter its directory.

### Install Dependencies

Make sure the F1TENTH Gym ROS simulator is installed and available in the workspace.

Install package dependencies using:

```bash
cd ~/F1Tenth-Repository

rosdep install --from-paths src --ignore-src -r -y
```

### Build the Workspace

```bash
cd ~/F1Tenth-Repository

colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

---

## Running the Simulation

The project uses two terminals.

### Terminal 1 — Launch the F1TENTH Simulator

```bash
cd ~/F1Tenth-Repository

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Wait until the simulator has started successfully.

<img width="1920" height="1080" alt="Screenshot from 2026-11-16 23-35-34" src="https://github.com/user-attachments/assets/b19b3932-1810-4372-b56f-568b7179325e" />

### Terminal 2 — Launch the Autonomous Controller

Open another terminal:

```bash
cd ~/F1Tenth-Repository

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run lidar_test autonomous_drive
```

The controller should start and begin processing LiDAR data.

### Expected ROS 2 Node

```text
/disparity_controller
```

### Expected Topics

```text
/scan
/ego_racecar/odom
/drive
```

<img width="765" height="242" alt="Screenshot from 2026-11-16 23-37-35" src="https://github.com/user-attachments/assets/9e0e7441-ba5d-4930-a320-86f1cf7eb9e1" />

### Demo Video

https://github.com/user-attachments/assets/57948253-24d6-4502-bc36-123e9b73cc88

---

## Changing the Track

The simulator uses `sim.yaml` to determine which map is loaded.

The current configuration uses the Zandvoort map.

Example configuration:

```yaml
map_path: '/home/<YOUR_WORKSPACE>/src/f1tenth_gym_ros/maps/Zandvoort_map'
map_img_ext: '.png'
```

To use another track:

1. Download or locate the desired map archive.
2. Extract the archive.
3. Copy the desired map files into the simulator's `maps` directory.
4. Update the `map_path` parameter in `sim.yaml`.
5. Restart the simulator.

The `map_path` value refers to the map file prefix, without the `.png` extension.

For example, to use another map named `BrandsHatch`:

```yaml
map_path: '/home/<YOUR_WORKSPACE>/src/f1tenth_gym_ros/maps/BrandsHatch'
map_img_ext: '.png'
```

Make sure the corresponding map image and YAML configuration are present in the simulator's map directory.

**Important:** Replace the example path with the actual location of your workspace and map files.

---

## Configuration

The simulator configuration is located in:

```text
config/sim.yaml
```

The configuration file contains settings related to:

- Vehicle namespaces.
- LiDAR and odometry topics.
- Simulator map selection.
- Number of simulated agents.
- Vehicle starting positions.
- Keyboard teleoperation.

Users can modify these settings according to their simulation requirements.

---

## Testing and Evaluation

The controller is evaluated in the F1TENTH Gym simulation environment.

The main evaluation criteria are:

| Evaluation Metric | Objective                                             |
| ----------------- | ----------------------------------------------------- |
| Consecutive laps  | Complete the required laps.                           |
| Collisions        | Avoid collisions with track boundaries and obstacles. |
| Lap completion    | Successfully navigate the selected track.             |
| Lap time          | Minimize lap completion time.                         |
| Driving stability | Maintain consistent autonomous driving.               |

### Testing Procedure

1. Launch the F1TENTH simulator.
2. Start the autonomous controller.
3. Allow the vehicle to drive around the selected track.
4. Observe whether the vehicle completes laps without collisions.
5. Record lap count and lap time.
6. Repeat testing after adjusting controller parameters.

### Simulation Testing

(https://github.com/user-attachments/assets/57948253-24d6-4502-bc36-123e9b73cc88)

**\<INSERT SCREENSHOT SHOWING THE VEHICLE COMPLETING A LAP HERE>**

---

## Experimental Results

Experimental results are recorded during simulation testing.

The evaluation focuses on lap completion time, vehicle speed, and driving consistency.

### Lap Time Records

The lap timer records the performance of each completed lap.

The following table provides a format for documenting the experimental results.

| Lap | Lap Time (s) | Peak Speed (m/s) | Lowest Speed (m/s) |
| --: | -----------: | ---------------: | -----------------: |
|   1 |            — |                — |                  — |
|   2 |            — |                — |                  — |
|   3 |            — |                — |                  — |
|   4 |            — |                — |                  — |
|   5 |            — |                — |                  — |
|   6 |            — |                — |                  — |
|   7 |            — |                — |                  — |
|   8 |            — |                — |                  — |
|   9 |            — |                — |                  — |
|  10 |            — |                — |                  — |

### Performance Summary

| Metric                | Result |
| --------------------- | ------ |
| Total Laps Recorded   | —      |
| Fastest Lap           | —      |
| Fastest Lap Time      | —      |
| Average Lap Time      | —      |
| Highest Peak Speed    | —      |
| Lowest Recorded Speed | —      |
| Number of Collisions  | —      |

**\<INSERT TERMINAL SCREENSHOT SHOWING THE COMPLETE LAP TIMER SUMMARY HERE>**

**\<INSERT SIMULATION SCREENSHOT SHOWING THE VEHICLE AFTER THE TEST RUN HERE>**

**\<INSERT VIDEO OR GIF OF THE AUTONOMOUS VEHICLE COMPLETING LAPS HERE, IF AVAILABLE>**

---

## Limitations and Future Improvements

Potential areas for further development include:

- Improving gap selection in narrow sections of the track.
- Reducing unnecessary steering oscillations.
- Improving speed adaptation during sharp turns.
- Increasing stability at higher driving speeds.
- Improving obstacle detection and safety margins.
- Evaluating performance across multiple racing tracks.
- Comparing different controller configurations using lap time and collision statistics.

---

## Acknowledgements

This project uses the F1TENTH ecosystem and ROS 2 tools for autonomous vehicle development and simulation.

Thanks to the developers and contributors of the F1TENTH Gym ROS simulator and the open-source ROS 2 community.

---

## License

This project is distributed under the license specified in the repository's `LICENSE` file.

If you use or modify this project, please retain the relevant licenses and attribution notices for third-party components.
