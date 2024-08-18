# Alex - Search and Rescue Robotic Vehicle

## Description
Alex is a state-of-the-art robotic vehicle designed to enhance search and rescue operations during natural disasters or terrorist attacks. The robot is equipped with advanced technologies to navigate hazardous terrains, identify potential victims, and provide real-time visualization of its environment. By integrating sophisticated sensors and secure communication systems, Alex aims to improve the effectiveness and safety of rescue missions.

## Motivation
In emergencies, swift and effective search and rescue operations are crucial for saving lives. However, rescuers often face significant risks when navigating unstable environments and obstructed passageways. Alex addresses these challenges by providing a robotic solution capable of traversing dangerous terrains, detecting obstacles, and identifying potential victims with precision. This innovation aims to enhance the chances of survival for individuals trapped under debris while ensuring the safety of rescue personnel.

## Features
- **Visualization**: Equipped with a Light Detection and Ranging (LiDAR) sensor, Alex scans and detects surrounding obstacles. Integrated with the Robot Operating System (ROS) and ROS Visualization (RViz), it allows operators to visualize the environment in real-time and identify potential victim locations remotely.

- **Secure Remote Control**: Operators can control Alex's direction, distance, and speed remotely with precision. The robot uses Transport Layer Security (TLS) communication to ensure secure interactions with a remote Personal Computer (PC).

- **Collision Prevention**: An ultrasonic sensor automatically halts Alex's movement upon detecting obstacles, preventing collisions and ensuring safe navigation.

- **Victim Identification**: A color sensor distinguishes between green, red, and white objects, representing healthy individuals, injured persons, and non-victims, respectively.

- **Compact Design**: Alex's small size enables it to navigate through narrow passageways and inaccessible areas that are challenging for human rescuers.

## Tech Stack

- **Sensors**:
  - LiDAR
  - Ultrasonic Sensor
  - Color Sensor

- **Software**:
  - Robot Operating System (ROS)
  - ROS Visualization (RViz)
  - Transport Layer Security (TLS)

- **Hardware**:
  - Raspberry Pi (R-Pi)
  - Arduino
  - Motor Driver
  - Motor & Encoder
  - Ring LED
  - Remote Personal Computer (PC)

## Core Devices and Components

- **Raspberry Pi (R-Pi)**: Manages communication between Alex’s Arduino and the operator’s PC through Universal Asynchronous Receiver-Transmitter (UART) and TLS respectively. It also publishes LiDAR data for remote visualization on the ROS Network.

- **Remote PC**: Sends commands to Alex and visualizes its surroundings.

- **Arduino**: Controls key components onboard Alex, including motors, wheel encoder, ultrasonic sensor, color sensor, and Ring LED.

- **Motor Driver**: Amplifies the signal from the Arduino to power and control the motor.

- **Motor & Encoder**: The motor powers the robot's movement, while the encoder provides feedback on speed and direction, aiding precise control.

- **LiDAR**: Scans Alex's surroundings, aiding in victim identification and navigation.

- **Ultrasonic Sensor**: Measures distances to obstacles ahead to avoid collisions.

- **Color Sensor & Ring LED**: The color sensor detects object colors, while the Ring LED displays the identified color.

## Additional Resources
- **[Documentation](B05-1A%20Final%20Report.pdf)**

## Authors
- Ping Hui (@TPH777)
- Frederick (@frederickemerson)
- Wen Xi (@frederickemerson)
- Keng Jer (@teokj)
