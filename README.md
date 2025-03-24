#### _This ROS package, `test_fetch_control`, is designed to solve a robotics problems (Three of them) involving the Fetch robot in a Gazebo simulation environment_. 
---
### Task 1 : _Given a target pose in the base frame, move the fetch arm to that pose by giving it as the target in the base frame_

The primary task addressed here is : Given a target pose (x, y, z, roll, pitch, yaw) in the base frame (`base_link`), move the Fetch robot arm to that pose using the Gazebo simulation environment.

### Solution
The solution uses ROS Melodic, Gazebo, and MoveIt! to control the Fetch robot arm. A Python node (`task1_moveit.py`) is implemented to:
1. Initialize a ROS node and MoveIt! commander.
2. Define a target pose in the base frame.
3. Plan a trajectory using MoveIt! and execute it to move the arm.

#### Dependencies
- ROS Melodic
- `fetch_gazebo` (for simulation)
- `fetch_moveit_config` (for MoveIt! integration)
- Python packages: `rospy`, `moveit_commander`, `geometry_msgs`, `tf`

#### How to Run
1. **Set up the workspace**:
   ```
   cd ~/catkin_ws/src
   catkin_create_pkg test_fetch_control roscpp rospy std_msgs geometry_msgs moveit_commander
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
2. **Launch Gazebo**:
   ```
   roslaunch fetch_gazebo simulation.launch
   ```
3. **Launch MoveIt!**:
   ```
   roslaunch fetch_moveit_config move_group.launch
   ```
4. **Run the Node**:
   ```
   rosrun test_fetch_control task1_moveit.py
   ```
   or
   ```
   rosrun test_fetch_control task1_moveit.py --x 0.3 --y 0.1 --z 0.6 --roll 0.0 --pitch 0.0 --yaw 0.785
   ```

### Results for Task 1

The Fetch arm moves to the specified pose (e.g., x=0.5, y=0.2, z=0.7, roll=0.0, pitch=0.0, yaw=1.57) in the Gazebo simulation

![task1 done](https://github.com/user-attachments/assets/c3214dbc-2e86-4d01-9015-d1f735c00730)

#### Code Explanation
- **Task:** Move the Fetch arm to a target pose (x, y, z, roll, pitch, yaw) in the base frame.
- **Solution:** Created task1_moveit.py in the test_fetch_control package using MoveIt!.
- **Implementation:** Set base_link as the reference frame, parsed pose inputs via command-line arguments (defaults: 0.4, 0.0, 0.5, 0.0, 0.0, 1.57), and converted to quaternion.
- **Execution:** Used MoveIt!’s RRTConnect planner to generate and execute a trajectory in Gazebo.
- **Result:** Successfully moved the arm with logged confirmation, leveraging MoveIt!’s IK and planning.

For help
```
rosrun test_fetch_control task1_moveit.py --help
```
Will give following output: 
```
usage: task1_moveit.py [-h] [--x X] [--y Y] [--z Z] [--roll ROLL]
                       [--pitch PITCH] [--yaw YAW]

Move Fetch arm to a target pose using MoveIt!

optional arguments:
  -h, --help     show this help message and exit
  --x X          X position in meters (default: 0.4)
  --y Y          Y position in meters (default: 0.0)
  --z Z          Z position in meters (default: 0.5)
  --roll ROLL    Roll angle in radians (default: 0.0)
  --pitch PITCH  Pitch angle in radians (default: 0.0)
  --yaw YAW      Yaw angle in radians (default: 1.57)

```
Hence, to solve Task 1—moving the Fetch robot arm to a given target pose (x, y, z, roll, pitch, yaw) in the base frame—we developed a Python script (task1_moveit.py) within the test_fetch_control ROS package, utilizing MoveIt! for motion planning and execution. 

The script initializes a ROS node and MoveIt!, setting base_link as the reference frame to interpret the target pose relative to the robot’s fixed base, consistent with the problem’s requirement. 

It accepts the pose via command-line arguments (with defaults: x=0.4, y=0.0, z=0.5, roll=0.0, pitch=0.0, yaw=1.57), converts it to a quaternion, and uses MoveIt!’s RRTConnect planner to compute a trajectory, which is then executed in the Gazebo simulation. 

This approach leverages MoveIt!’s inverse kinematics and planning capabilities to ensure accurate, collision-free arm movement, verified through successful testing in Gazebo with logged feedback.

---

### Task 2 : _Use TF (ROS package) to compute what a given base frame pose would be in the camera frame. Then move the fetch arm to that pose in the camera frame._






