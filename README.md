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
5. We can cross verify by moving the robot and trying to reach same pose
   ```
   rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10
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

The primary task addressed here is : Given a target pose (x, y, z, roll, pitch, yaw) in the base frame (interpreted as ```odom``` for this task), compute its equivalent pose in the camera frame (```head_camera_rgb_optical_frame```) using TF, 
and then move the Fetch robot arm to that transformed pose in the Gazebo simulation environment.

### Solution
The solution uses ROS Melodic, Gazebo, and MoveIt! to control the Fetch robot arm. A Python node (`task2_moveit.py`) is implemented to:
1. Initialize a ROS node, MoveIt! commander, and TF listener.
2. Define a target pose in the ```odom``` frame (base frame for Task 2).
3. Transform the pose to the ```head_camera_rgb_optical_frame``` using TF.
4. Plan and execute a trajectory with MoveIt! to move the arm to the transformed pose.


#### Dependencies
- ROS Melodic
- `fetch_gazebo` (for simulation)
- `fetch_moveit_config` (for MoveIt! integration)
- Python packages: `rospy, moveit_commander, geometry_msgs, tf, tf.transformations`

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
   rosrun test_fetch_control task2_moveit.py
   ```
   or
   ```
   rosrun test_fetch_control task1_moveit.py --x 0.5 --y 0.4 --z 0.6 --roll 0.0 --pitch 0.0 --yaw 0.785
   ```

### Results for Task 2

The Fetch arm moves to the transformed pose in the `camera frame`, derived from the input pose in `odom` (e.g., x=0.5, y=0.5, z=0.5, roll=0.0, pitch=0.0, yaw=0.0), as computed by TF and executed in the Gazebo simulation.

![task2_fetch](https://github.com/user-attachments/assets/ab909f18-c4bf-4d5a-ac6f-da6ce64076a4)


#### Code Explanation
- **Task:** Compute a given pose in the base frame (odom) in the camera frame (head_camera_rgb_optical_frame) using TF, then move the Fetch arm to that pose.
- **Solution:** Created task2_moveit.py in the test_fetch_control package using MoveIt! and TF.
- **Implementation:** Set odom as the input base frame and head_camera_rgb_optical_frame as the target frame.
- Parsed pose inputs via command-line arguments (defaults: 0.5, 0.5, 0.5, 0.0, 0.0, 0.0).
- Used TF’s transformPose to convert the odom pose to the camera frame, logging both poses with Euler angles.
- Configured MoveIt! with the camera frame as the reference and used the RRTConnect planner to move the arm.
- **Execution:** The script transforms the pose and executes the arm movement in Gazebo, reflecting the robot’s position in odom.
- **Result:** Successfully moved the arm to the transformed pose, verified through Gazebo and logged output.

For help
```
rosrun test_fetch_control task2_moveit.py --help
```
Will give following output: 
```
usage: task2_moveit.py [-h] [--x X] [--y Y] [--z Z] [--roll ROLL]
                       [--pitch PITCH] [--yaw YAW]

Move Fetch arm to a pose in odom frame, transformed to camera frame

optional arguments:
  -h, --help     show this help message and exit
  --x X          X position in meters
  --y Y          Y position in meters
  --z Z          Z position in meters
  --roll ROLL    Roll angle in radians
  --pitch PITCH  Pitch angle in radians
  --yaw YAW      Yaw angle in radians
```

Hence, to solve Task 2—computing a given base frame pose in the camera frame and moving the Fetch arm there—we developed a Python script (task2_moveit.py) within the test_fetch_control ROS package, utilizing TF for transformation and MoveIt! for motion control. 

The script initializes a ROS node, MoveIt!, and a TF listener, interpreting the base frame as odom (a world-like frame) and the target as head_camera_rgb_optical_frame. 

It accepts a pose via command-line arguments (defaults: x=0.5, y=0.5, z=0.5, roll=0.0, pitch=0.0, yaw=0.0), transforms it using TF, and uses MoveIt!’s RRTConnect planner to execute the arm movement in the camera frame within Gazebo. 

This approach ensures the arm reaches the exact transformed pose, verified through simulation and logged feedback.

---

### Task 3 : _Detect the pose of an AR marker from the camera and move the fetch arm to that pose_

The primary task addressed here is : Detect the pose of a custom ArUco marker using the Fetch robot’s head camera in a Gazebo simulation, 
and then move the Fetch arm to the detected marker’s pose in the camera frame (`head_camera_rgb_optical_frame`). 
The solution uses `ar_track_alvar` for marker detection and MoveIt! for arm control.

### Solution

The solution uses ROS Melodic, Gazebo, `ar_track_alvar`, and MoveIt! to achieve the task. It consists of two components:
1. A launch file (`aruco_detect.launch`) to detect the ArUco marker using `ar_track_alvar` and publish its pose on topic `/ar_pose_marker`.
2. A Python node (`task3_moveit.py`) to subscribe to the marker’s pose and move the Fetch arm to that pose using MoveIt!.


#### Dependencies
- ROS Melodic
- `fetch_gazebo` (for simulation)
- `fetch_moveit_config` (for MoveIt! integration)
- `ros-melodic-ar-track-alvar` (for ArUco marker detection)
- `fetch_msgs` (for Fetch robot compatibility)
- Python packages: `rospy, moveit_commander, geometry_msgs`

#### How to Run
1. **Set up the workspace**:
   ```
   cd ~/catkin_ws/src  
   catkin_create_pkg test_fetch_control roscpp rospy std_msgs geometry_msgs moveit_commander ar_track_alvar
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Generate the Custom ArUco Marker**:
   ```
   cd ~/catkin_ws/src/test_fetch_control  
   mkdir -p markers  
   cd markers`  
   rosrun ar_track_alvar createMarker 0
   ```  
   This generates `MarkerData_0.png`. Print it with the black square measuring 4.4 cm x 4.4 cm (as specified in the launch file).
   **Note:** I used pre-existing Marker from a github repository added into Gazebo Models. So, I added the Marker manually in the Gazebo Environment. (Marker 0)

4. **Launch Gazebo**:
   
   ```
   roslaunch fetch_gazebo simulation.launch
   ```

5. **Launch MoveIt!**:
   
   ```
   roslaunch fetch_moveit_config move_group.launch
   ```

6. **Run the Detection Node**:
   
   ```
   roslaunch test_fetch_control aruco_detect.launch
   ```

7. **Run the Arm Movement Node**:
      ```
      rosrun test_fetch_control task3_moveit.py
      ```

#### Results for Task 3

The `ar_track_alvar` node detects the custom ArUco marker (ID 1) using the Fetch robot’s head camera in the Gazebo simulation, publishing its pose on `/ar_pose_marker`. 
The `task3_moveit.py` node subscribes to this pose and moves the Fetch arm to the marker’s position in the `head_camera_rgb_optical_frame`, verified through Gazebo simulation.

![task3](https://github.com/user-attachments/assets/7fe9f968-504e-42e9-839d-127f21a890f4)

#### Code Explanation
- **Task:** Detect the pose of an ArUco marker using the Fetch robot’s head camera and move the arm to that pose.
- **Solution:** Created `aruco_detect.launch` to run `ar_track_alvar` for marker detection, and `task3_moveit.py` to move the arm using MoveIt!.
- **Implementation:** Generated a marker with ID 1 using `rosrun ar_track_alvar createMarker 1`, which uses the `DICT_6X6_250` dictionary (default for `ar_track_alvar`).  
- Configured `aruco_detect.launch` to use the Fetch robot’s head camera topics (`/head_camera/rgb/image_raw` and `/head_camera/rgb/camera_info`) and set the marker size to 4.4 cm.  
- Set the output frame to `/base_link`.  
- The `task3_moveit.py` script subscribes to `/ar_pose_marker`, extracts the marker’s pose, and uses MoveIt! with the RRTConnect planner to move the arm to the detected pose in the camera frame.
- **Execution:** The launch file runs the `individualMarkers` node from `ar_track_alvar`, which detects the marker and publishes its pose. The Python node processes this pose and executes the arm movement in Gazebo.
- **Result:** Successfully detected the marker and moved the arm to its pose, verified through Gazebo simulation and logged output.

