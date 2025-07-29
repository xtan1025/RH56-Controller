# RH56 Advanced Hand Controller

## 1. Project Overview

This project contains a Python script (`controller.py`) for controlling the RH56 dexterous hand via a serial port. The script encapsulates the low-level communication protocol and provides a high-level `RH56Hand` Python class, making it easier for developers to implement complex control logic.

Currently, the project features a **Adaptive Force Control** function that can adjust force thresholds while dynamically changing finger positions to achieve a preset contact force.

## 2. Core Features

- **Basic Control**:
  - Set/read the angle for all six degrees of freedom (five fingers + thumb rotation).
  - Set/read the movement speed for each finger.
  - Set/read the force control threshold for each finger (unit: grams).
- **Sensor Reading**:
  - Real-time reading of the pressure sensor for each finger (unit: grams).
- **Force Sensor Calibration**:
  - Provides an interactive calibration routine to calibrate the force sensors before precise control operations.
- **Advanced Control Logic**:
  - **Adaptive Force Control (`adaptive_force_control`)**: This is an advanced control mode with the following characteristics:
    1. **Position-Force Coordinated Control**: Can simultaneously move fingers to a target **angle** and have them reach a target **contact force**.
    2. **Step-wise Adjustment**: Gradually moves fingers to the target position instead of all at once, making the control process smoother and more stable.
    3. **Intelligent Force Adjustment**: During movement, it dynamically adjusts the force control threshold based on the difference between the current force reading and the original target.

## 3. Setup and Installation

### Hardware
- RH56 Dexterous Hand
- USB-to-Serial adapter to connect the hand to the computer

### Software
- Python 3
- `pyserial` library
- `numpy` library


## 4. Configuration

Before running the script, you need to modify two key parameters at the **bottom** of the `controller.py` file, inside the `if __name__ == "__main__":` block, according to your setup:

1.  **Serial Port (`port`)**:
    -   Find the line `hand = RH56Hand(...)`.
    -   Change the `port` parameter to the actual serial port recognized by your computer.
        -   **Windows**: e.g., `COM3`, `COM4`
        -   **macOS/Linux**: e.g., `/dev/tty.usbserial-xxxx` or `/dev/ttyUSB0`

2.  **Hand ID (`hand_id`)**:
    -   In the same line, modify the `hand_id` parameter.
        -   **1**: Right Hand
        -   **2**: Left Hand

**Example:**
```python
if __name__ == "__main__":
    # Modify the parameters here based on your hardware connection
    hand = RH56Hand(port="/dev/tty.usbserial-1130", hand_id=1) 
    ...
```

## 5. Usage

The script can be run directly to start the pre-configured **Adaptive Force Control** demonstration.

### Steps to Run
1.  **Connect Hardware**: Ensure the dexterous hand is correctly connected to the computer and powered on.
2.  **Modify Configuration**: Correctly configure the serial port and hand ID as described in the previous section.
3.  **Execute Script**: Run the following command in your terminal:
    ```bash
    python controller.py
    ```
4.  **Start Calibration (Optional)**:
    -   By default, the script first runs `demonstrate_force_calibration`.
    -   You will see the prompt `Press Enter to start calibration...`. Press Enter to begin. The calibration process takes about 15 seconds.
    -   If you do not need to calibrate, you can comment out the `demonstrate_force_calibration(...)` line in the `__main__` block.
5.  **Observe Adaptive Force Control**:
    -   After calibration, the script will automatically start the `adaptive_force_control` routine.
    -   You will see real-time output in the terminal showing each finger's **current angle**, **current force reading**, **original target force**, and the **action taken** for each iteration.
    -   The program will finish after reaching the targets or the maximum number of iterations and will print a final summary report.

## 6. Key Methods (API)


---
`force_set(thresholds: List[int])`
- **Function**: Directly sets the force control thresholds for the 6 fingers.
- **Parameters**: `thresholds` - A list of 6 integers, with each value ranging from 0-1000g.

---
`angle_set(angles: List[int])`
- **Function**: Sets the target angles for the 6 fingers.
- **Parameters**: `angles` - A list of 6 integers, with each value ranging from 0-1000.

---
`force_act() -> Optional[List[int]]`
- **Function**: Reads and returns the current force sensor readings for the 6 fingers (unit: grams).
- **Returns**: A list of 6 integers, or `None` if the read fails.

---
`angle_read() -> Optional[List[int]]`
- **Function**: Reads and returns the current angle positions for the 6 fingers.
- **Returns**: A list of 6 integers, or `None` if the read fails.

---
`adaptive_force_control(target_forces: List[int], target_angles: List[int], step_size: int = 50, max_iterations: int = 20)`
- **Function**: Executes the advanced adaptive force control routine.
- **Parameters**:
  - `target_forces`: List of target contact forces (unit: grams).
  - `target_angles`: List of target angles.
  - `step_size`: The angle step for each iteration.
  - `max_iterations`: The maximum number of iterations.
- **Returns**: A dictionary containing detailed results and history.

---
`demonstrate_force_calibration(port: str, hand_id: int)`
- **Function**: Starts an interactive force sensor calibration routine. It is recommended to run this before performing precision tasks. 
