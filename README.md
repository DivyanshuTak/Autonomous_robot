# Autonomous_robot
The robot is a 3 wheel omni drive robot which consists of sensors like Inertial Measurement Unit (IMU), rotary
encoders, proximity sensors and line detection infrared sensors. The data from all the sensors
are fused to get the position of the robot and detect the obstacles in the path. This fused data is
then fed to the motion planning algorithm, which included both dead reckoning and line following
methods along with RRT* algorithm to make the robot move from one point to another in smooth trajectory. 
The specific tasks are performed by interfacing the mechanical actuators to the robot. The whole project
is implemented on the Arm Cortex Microcontroller (LPC1769), and the program was written in
Embedded C and C++. The code is implemented in a modular fashion, with a separate header
and C++ file for the collection of data from each sensor. All the sensor data is used using
interrupts in the main algorithm. The main algorithm is finally employed to provide high level
commands which are implemented by the drive controllers of the robot using PID control. I also
performed code optimization to improve the performance of the program and reduce the latency
in the system.
