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
Run D* Lite alogithm:
Open third terminal
- **Run robot Dstarlite**
  ```sh
  ros2 run robot_planner dstar
  ```
    
