# Robust Trajectory Tracking for Quadcopter UAVs

The objective of this project is to develop a robust control scheme to enable a quadrotor to track
desired trajectories in the presence of external disturbances.The control design is tested on the Crazyflie 2.0 platform. Crazyflie is a quadrotor
that is classified as a micro air vehicle (MAV), as it only weighs 27 grams and can fit in a hand.

## Crazyflie 2.0 Setup in Gazebo

To set up the Crazyflie 2.0 quadrotor in Gazebo, we need to install additional ROS dependencies
for building packages as below:
```
sudo apt update
```
```
sudo apt install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
```
```
sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
```
```
sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
```
```
rosdep update
```
```
sudo apt-get install ros-noetic-ros libgoogle-glog-dev
```
## Instruction for setting the Enviroment 

**Clone this repo to create a new ROS workspace for Sliding Mode Controller**

```
git clone <GitHub Repository SSH>
```
**Initialize catkin workspace**

```
cd ~/Robust-Trajectory-Tracking-for-Quadcopter-UAVs/src
```

```
catkin_init_workspace
```
```
cd ~/Robust-Trajectory-Tracking-for-Quadcopter-UAVs
```
```
catkin init
```

**Downloading necessary the ROS packages:**

```
cd ~/Robust-Trajectory-Tracking-for-Quadcopter-UAVs/src
```
```
git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
```
```
git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
```

**We need to build the project workspace using python_catkin_tools, therefore we need to configure it:**
```
cd ~/Robust-Trajectory-Tracking-for-Quadcopter-UAVs
```
```
rosdep install --from-paths src -i
```
```
rosdep update
```
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
```
```
catkin build
```
## Running the Project

**Before running the project, make sure to change the directory to store log file for visualization**

src/**main.py** -> Change directory on line **101**

**To spawn the quadrotor in Gazebo, we can run the following launch file:**

```
roslaunch rotors_gazebo crazyflie2_without_controller.launch
```
**To start the controller, we can run the following script file:**
```
rosrun Robust-Trajectory-Tracking-for-Quadcopter-UAVs main.py
```

## Simulation Results

### **Trajectory tracking**

Following plot displays the desired trajectory to be tracked in green and the actual trajectory followed by the quadcopter in blue and their deviation.

![trajectory](https://user-images.githubusercontent.com/72921304/208376503-250e9ea8-881f-4274-8a5b-c75674691157.png)

### **Gazebo simulation**

![ezgif com-gif-maker](https://user-images.githubusercontent.com/72921304/208376568-bcdbbdfb-691d-4f91-98c8-67eb480cf2fb.gif)
