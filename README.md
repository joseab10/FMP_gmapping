# Full Map Posterior Grid-based Fast-SLAM

This project extends the OpenSLAM GMapping framework and it's ROS wrapper to support computing Full Map Posteriors
besides the Most-Likely Map according to the paper 
***["Closed-Form Full Map Posteriors for Robot Localization with Lidar Sensors"](https://arxiv.org/abs/1910.10493)***
by Lukas Luft, Alexander Schaefer, Tobias Schubert and Wolfram Burgard.

## Usage

### Installation

#### Downloading
In a computer with ROS installed, download or clone the repository:
```console
foo@bar:-$ git clone https://github.com/joseab10/FMP_gmapping
```

#### Compiling
In a command line terminal:
1. Ensure that the ROS environment is properly set
    ```console
    foo@bar:-$ source /opt/ros/${ROS_DISTRO}/setup.bash
    ```
1. Change directory to repository
    ```console
    foo@bar:-$ cd <dir>/FMP_gmapping
    ```
1. Compile using Catkin
    ```console
    foo@bar:-$ catkin_make
    > Base path: /…/FMP_gmapping
    > Source space: /…/FMP_gmapping/src
    > Build space: /…/FMP_gmapping/build
    > Devel space: /…/FMP_gmapping/devel
    > Install space: /…/FMP_gmapping/install
    > ####
    > #### Running command: "cmake …
    > /…/FMP_gmapping/src
               ⋮
    > [ 98%] Built target slam_gmapping_replay
    > [100%] Linking CXX executable /…/FMP_gmapping/devel/lib/gmapping/slam_gmapping
    > [100%] Built target slam_gmapping
    ```
1. Set the ROS environment to the FMP_gmapping workspace
    ```console
    foo@bar:-$ source devel/setup.bash
   
### Running

#### Test Datasets
The workspace includes several benchmark datasets in the `data/` directory, as well as preconfigured ***roslaunch***
files in `src/fmp_gmapping/launch/` to start using it right away.

In order to start a launch file, with the environment properly set to the FMP_gmapping workspace, run:
```console
foo@bar:-$ roslaunch fmp_gmapping <launch_file>
```
where:
   * `<launch_file>` is one of the many files in `src/fmp_gmapping/launch/`

This will start the following processes and nodes:

  * ***roscore:*** ROS Master process
  * ***gmapping*** **node:** Main Grid-based SLAM node
  * ***rosbag play*** **node:** Simulation node
  * ***rviz:*** Program for the visualisation of the step-by-step results 
  
##### Example:
The following command runs the preconfigured SLAM algorithm on the corrected odometry dataset of building 079 in the
University of Freiburg:
```console
foo@bar:-$ roslaunch fmp_gmapping fr_079_corr.launch
```

#### Standalone node
In order to use FMP_Gmapping as a standalone node for your own projects, simply run it as any other ROS node with your required remappings and parameters:
```console
foo@bar:-$ rosrun gmapping slam_gmapping
```

For more details on the specifics, check the `README.md` file in the *slam_gmapping* node source directory. 
