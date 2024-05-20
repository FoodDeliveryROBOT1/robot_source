Robot Run command:
Open the first terminal
- **Launch robot communicate with Motor Driver via CanBus communication, robot localization and turn on the Ydlidar:**
    ```sh
    ros2 launch robot_can robot.launch.py
    ```

Create a map:
Open the sencode terminal 
- **Launch robot mapping:**
    ```sh
    ros2 launch robot_can mapping.launch.py
    ```
Run D* Lite alogithm for robot global path planner:
Open third terminal
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
  
    
