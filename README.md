# AR SCARA Robot Arm
SCARA Robot Arm created using C++ and the OpenCV library to display in AR.
Starting on this project I first learned the necessary fundamental concepts for creating a coordinate system and connecting joints for a robot. This began with learning 2D and 3D coordinate system transformations. I used the properties of homogeneous transformation matrices to rotate and translate the robot.

![image](https://github.com/ZSheppard/AR-SCARA-Robot-Arm/assets/77692349/a5fcd17e-9cbb-45d1-ac60-d94f50a99c86)

After learning the basics I created a model of the robot arm within a virtual camera. Using sliders, a user can adjust both the camera position and position of the robots joints.

Control of the joints is divided into two methods: Forward Kinematics and Inverse Kinematics.
Forward Kinematics means that each joint of the robot arm can be controlled individually.
Inverse Kinematics controls the end effector (or the tip of the arm) position and has the joints of the robot follow where the user positions the end effector. Inverse Kinematics requires computing equations that represent the joint angles of the robot and wha they have to be to correspond with the desired end effector position. These equations can be large with many, many trig variables which is why they usually require simplification.

![image](https://github.com/ZSheppard/AR-SCARA-Robot-Arm/assets/77692349/a7ea0fe7-9c27-4749-a070-c7db83a744d0)


Using a printed ChArUco grid as a canvas, I could identify specific points on the paper and set up my coordiante system. The robot would then appear at one of the corners acting as the 0,0 point.

![image](https://github.com/ZSheppard/AR-SCARA-Robot-Arm/assets/77692349/4367aeab-7b58-46b7-8801-952e8156140e)

