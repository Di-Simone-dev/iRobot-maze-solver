# Comandi per la simulazione ROS
1. ## UNDOCK
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'UNDOCK'}"
    ```

2. ## DOCK
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'UNDOCK'}"
    ```

3. ## MODE_A (PLEDGE)
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'MODE_A'}"
    ```

4. ## MODE_B (TREMAUX)
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'MODE_B'}"
    ```

5. ## SOLVE
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'SOLVE'}"
    ```

6. ## FORWARD
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'FORWARD'}"
    ```

7. ## BACKWARD
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'BACKWARD'}"
    ```

8. ## ROTATE_LEFT
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'ROTATE_LEFT'}"
    ```

9. ## ROTATE_RIGHT
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'ROTATE_RIGHT'}"
    ```

10. ## START
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'START'}"
    ```

11. ## STOP
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'STOP'}"
    ```

12. ## PAUSE
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'PAUSE'}"
    ```

13. ## RESUME
    ```bash
    ros2 topic pub --once /command custom_msg/msg/Command "{command: 'RESUME'}"
    ```
