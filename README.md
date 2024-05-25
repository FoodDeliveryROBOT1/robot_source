## Project About the Motion Planning, Control, Localization, and Mapping of a differential wheeled mobile robot.
The project included the process of global path planning with the D* lite algorithm which is the incremental heuristic search best for the dynamic environment, trajectory tracking with two methods (Pure Pursuit Controller and Nonlinear Model Predictive Control NMPC). Mapping Generation using Slam Toolbox. Localization using the Extended Kalman Filter with the Monte Carlo Localization MCL.

The command code below is used to run for the launch robot controller, Mapping, Localization, Path Planning, and Trajectory Tracking.

Robot Run command:
Open the first terminal
- **Launch robot communicate with Motor Driver via CanBus communication, robot localization and turn on the Ydlidar:**
    ```sh
    ros2 launch robot_can robot.launch.py
    ```
After launch the robot, diver will be able to controlled, the lidar will on, and the IMU will works. In this case, the robot can be control through the teleop keyboard. But attention the velocity must be decrease from the 0.5m/s to 0.1m/s.
    ```
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

Create a map:
Open the second terminal 
- **Launch robot mapping:**
    ```sh
    ros2 launch robot_can mapping.launch.py
    ```
Run D* Lite algorithm for robot global path planner:
Open the third terminal
- **Run robot Dstarlite**
  ```sh
  ros2 run robot_planner dstar
  ```
Run robot path tracking:
Open another terminal
- **Run Pure Pursuit Controller**
    ```sh
    ros2 run robot_planner pps
    ```
- **Run Nonlinear Model Predictive Control NMPC**
  ```sh
  ros2 run robot_planner nmpc
  ```

To visualize the process:
Open another terminal
- **Run Rviz**
  ```sh
  rviz2
  ```

    
