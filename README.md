
# Robot Arm

This project involves a robotic arm designed to sort pieces based on their color and position. The robotic arm uses multiple sensors to detect and handle pieces accurately. Below are the components and details about how the system works.

## Components
The circuit is shown below:

![Main_circuit](https://github.com/user-attachments/assets/bbec209b-13a9-4dc1-a4e5-967e94243ea1)

The final robot assembled is:

![Robotarm](https://github.com/user-attachments/assets/9ffb8aba-ae2d-4cca-8e13-c6d694672e9c)

The servo motors are organized as follows:

![Equations2](https://github.com/user-attachments/assets/9be68ed8-02a3-4eb9-a315-55bdad69f60c)

![Equations3](https://github.com/user-attachments/assets/ba1f9c61-c00a-43e2-845d-e16fe89aa68b)

![Equations4](https://github.com/user-attachments/assets/8b54c8c1-3564-4e37-bff9-0055ed30f961)

The grid setup for positioning objects is as shown below:

![Equations1](https://github.com/user-attachments/assets/2562e7ec-5642-4453-9b79-32f5bd8020d6)

## Sensors
The system incorporates the following sensors:

- **RGB Color Sensor (TCS34725)**: Used to detect the color of the pieces.
- **Time-of-Flight (ToF) Distance Sensor (VL53L0X)**: Used to measure the distance and locate the pieces for precise handling.

## Robot Modes
The robotic arm operates in three modes, which can be activated via keyboard input. These modes allow the robot to interact with the pieces in different ways:

1. **Mode 1**: In this mode, the robotic arm picks up a single object from a predefined position in a grid and places it in a designated box based on the object's color. The user is prompted to enter the x and y coordinates of the piece in the grid, and the robot will calculate the required polar coordinates and servo motor angles.

2. **Mode 2**: This mode works similarly to Mode 1, but allows for the sorting of multiple objects. The user inputs how many pieces to sort, and the robot will sequentially pick and place each piece in the correct box based on its color. The pieces are arranged in a 3x3 grid, and the robot moves through each coordinate to complete the sorting.

3. **Mode 3**: This mode utilizes the ToF sensor to detect pieces regardless of their position in the grid. The robot will search for the pieces, pick them up, check the color, and place them in the corresponding box.

## Equations
Here are some of the key equations used in the system:

1. **Servo Motor Angle to Duty Cycle Calculation**:
    - Placeholder for the servo angle calculation equation.

2. **Grid to Polar Coordinates**:
    - Placeholder for the grid to polar coordinates transformation.

3. **Robot Kinematics**:
    - Placeholder for the robot kinematics equations related to positioning the arm.

The full equations and their corresponding code can be found in the project code file.

## Conclusion
This robotic arm project successfully achieves the goal of picking up and sorting pieces based on their color and position. The arm operates in three distinct modes and uses multiple sensors for precise operation. The code is structured to handle the motor control, sensor readings, and piece sorting operations efficiently.
