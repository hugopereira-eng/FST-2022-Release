# :robot: Autonomous System 

## Table of Contents
- [Description](#description)
- [Setup](#setup)
- [Usage](#usage)
- [Simulation](#simulation)
- [Troubleshooting](#troubleshooting)
- [Tests](#tests)
- [Versioning](#versioning)
- [Documentation](#documentation)
- [FAQ](#faq)
- [Team](#team)
- [License](#license)
- [Acknowledgments](#acknowledgments)


## Description
This is the catkin workspace used to build the ROS packages implementing the autonomous software modules. The modules are divided in:
* common - containing CAN communication nodes and high level management of the system. Also ROS messages definition.
* control - containing control algorithms and path planning.
* estimation - containing odometry and SLAM.
* perception - containing the camera and lidar pipelines for pose acquisition of the cones and also the sensor fusion package.

### Features
1.  Cone identification
    1. [x] Pointcloud clusters position
    2. [x] Camera cone identification
    3. [x] Pointcloud cone identification
    4. [x] LiDAR cone color identification 
    5. [~] Camera cone distance estimation
1.  Sensor Fusion
    1. [x] Camera-Lidar calibration
    2. [x] Validate cones using the matches with the camera
    3. [x] Camera-Lidar synchronization
1.  Trajectory
    1. [x] Generate trajectory based on sensor fusion validated cone detections
    2. [x] Generate trajectory using uncolored cones
    3. [x] Build a decision tree of possible paths
    4. [x] Implement cost function
1.  Control   
    1. [ ] Lateral Controller
    2. [x] Longitudinal Controller
1.  Common   
    1. [x] Launcher
    2. [x] Mission Status Management
    3. [x] Mission Conclusion Identification


## Setup
To install this software you can simply download and run the script *install_pipeline.sh* which is in the *autonomous_system* directory, without cloning the repository. This script creates a directory called *fst* in the directory where it is ran, creating two subdirectories for the simulator and the pipeline respectively. The script installs needed dependencies, clones both the simulator and the pipeline repositories, and compiles both softwares. The user must have access to gitlab with an ssh key in order for the script to work properly.

Alternatively, you can follow these instructions to get a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites
What you need to install and how to install them:
- [Ubuntu 18.04](http://releases.ubuntu.com/18.04/)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [CUDA](https://docs.google.com/document/d/1kpP-cBP_7RJT-jDUoM27uVD4CV5jGBRIbmtHrDZ7DRw)
(for Nvidia GPUs only). Follow the section about CUDA installation.
- [FSSIM](https://gitlab.com/projectofst/software10d/-/tree/dev/autonomous-system#install-the-simulator)

### Installing

A step by step series of examples that tell you how to get a development env running

- Get the repository

```
$ git clone https://gitlab.com/projectofst/software10d.git
```
or, if you have a SSH key
```
$ git clone git@gitlab.com:projectofst/software10d.git
```
- Install velodyne package
```
 $ sudo apt-get install ros-melodic-velodyne
```
- Install CGAL Library: 
```
$ sudo apt-get install libcgal-dev
```
- Install Qt interface dependencies:
```
$ sudo apt-get install libcgal-qt5-dev
```
- Go the directory of the repository:
```
$ cd software10d/autonomous-system/
```
- Install dependencies:
```
$ rosdep install --from-paths src --ignore-src --rosdistro melodic -y
```
- Install pip3:
```
sudo apt install python3-pip
```
- Add python dependencies:
```
$ pip3 install -r requirements.txt
```
- Source the fst_environment.sh file. This will source the devel/setup.bash and all the aliases.
```
$ echo "source /path/to/software10d/autonomous-system/fst_environment.sh" >> ~/.bashrc
```
- Run the environment file
```
$ ./fst_environment.sh
```
- Source bashrc
```
$ source ~/.bashrc
```
- Compile the code
```
$ asmake
```


## Usage

- To run the full pipeline use:

```
$ roslaunch common_meta_pkg complete_pipeline.launch
```

- To execute a mission run (replace MISSION with the mission name you want to execute):

```
$ roslaunch common_meta_pkg MISSION.launch
```

- To only launch a subpart of the pipeline use: (we show the example for
perception here). Just replace the word perception by estimation or control:

```
$ roslaunch perception_meta_pkg perception_complete.launch
```

- To only launch a single node use for example:

```
$ roslaunch lidar lidar_cone_detector
```

## Simulation

### Install the simulator

Follow these steps to install our version of the AMZ Simulator [FSSIM](https://gitlab.com/projectofst/fssim):

- Install python catkin tools (if you haven't already):
```
$ sudo apt install python-catkin-tools
```

- Create a new directory (ex. simulator), outside of any repository folder or catkin workspace,
and a subdirectory named "src":
```
$ mkdir -p simulator/src
```

- Navigate to the created directory and initialize the workspace with *catkin init*:
```
$ cd simulator
$ catkin init
```

- Clone the [FSSIM](https://github.com/AMZ-Driverless/fssim) repository in the "src" directory:
```
$ cd src
$ git clone https://gitlab.com/projectofst/fssim
$ cd fssim
$ git checkout melodic-devel
```

- Update dependencies:
```
$ ./update_dependencies.sh
```

- Build the workspace:
```
$ catkin build
```

- Source the workspace, and/or add this to your bashrc (make sure that this is sourced **after** the source of fst_environment.sh):
```
$ source ../../devel/setup.bash
```

- Run the simulator:
```
$ roslaunch fssim auto_fssim.launch
```

**Note:** The RVIZ window will start and the terminal will inform you of what is
happening. It might take a little while to load for the first time. You might need to untick and tick *FSSIM Track* and *RobotModel* in RVIZ in order to load the STL files.


### Run a simulation

To run a simulation of our code do:

```
$ roslaunch fst_interface fssim.launch
```
Then, run the autonomous system so that the car starts moving. You can run a specific mission by calling the respective launch file. Replace MISSION with the mission you want to simulate.
**Note:** you need to load into the simulator the track according to the mission you selected. <br />
The file to do so is in "autonomous-system/src/simulation/fst_interface/fssim_config/simulations/simple_simulation.yaml

```
$ roslaunch simulation_meta_pkg simul_MISSON.launch
```

## Troubleshooting

Follow this [link](https://docs.google.com/document/d/1aZG1GkAg53vkCczvlkKr51tEAu6BMAsG2f1cqD1AGvc/edit) for the Troubleshooting guide.

## Tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```
### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Versioning

We use [Gitlab Releases](https://gitlab.com/help/user/project/releases/index.md#creating-a-release) for versioning, releasing a new version whenever a major module pipeline upgrade/fix has been launched. We also use [Git Tags](https://gitlab.com/help/university/training/topics/tags.md) to mark important code checkpoints, *e.g.* new features. For the code versions available, see the [releases on this repository](https://gitlab.com/projectofst/software10d/-/releases). 

We have two protected branches:

- *master*: merge request is accepted whenever a given release has been validated with representative tests in the car.
- *dev*: merge request is accepted whenever a new feature/fix has been thoroughly tested in simulation or equivalent, and code has been reviewed by peers.

### Git Naming Conventions

We work with a monorep, *i.e* all the code that goes in the car, from any module of the autonomous system to low-voltage LED controller electronics, is in this repository. Therefore, it is of utmost importance that everyone uses appropriate naming conventions (we based ours on [this](https://gist.github.com/joshbuchea/6f47e86d2510bce28f8e7f42ae84c716)), so that it is easy to navigate within the repo. Keep in mind there are a lot of developers working on this repo simultaneously. Naming convention goes as follows:

- **Branch**: `<system>/<type>/<module>-<submodule>-<descriptionOfBranch>`
- **Commit**: `<type>(<system>): <subject>`

Any folder found [here](https://gitlab.com/projectofst/software10d/-/tree/dev) represents a system, *e.g* `autonomous-system`or `iib`. It represents every piece of hardware running software that goes in the car, *e.g.* Onboard PC and IIB PCB, respectively.

We use the following types of branches/commits:

- `feat`: introduces a new feature to the codebase.
- `fix`: patches a bug of our codebase.
- `docs`: changes to documentation.
- `style`: formatting, missing semi colons, etc; no production code change.
- `refactor`: refactoring production code, *e.g.* renaming a variable; no production code change.
- `test`: adding missing tests, refactoring tests; no production code change.
- `chore`: updating grunt tasks etc; no production code change.

The last part of the branch ID focuses specially on Autonomous System develoment since this encompasses several modules, which can be found [here](#description). Submodules are several and may change in time so they are not worth mentioning. Nonetheless, these are well-defined and should be clear to any developer working on this system.

Please use enlightening branch and commit descriptions!!

## FAQ
- **How do I do *specifically* so and so?**
    - No problem! Just do this.

---
## Documentation

## Team

* **Billie Thompson** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc
