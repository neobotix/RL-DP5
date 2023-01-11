# RL-DP5
Software package for RL-DP5 Igus arm with extended DoF's

This package was designed to work with the mobile manipulator mm-400 mounted with RL-DP5 with the linear actuator. This package provides all the necessary ros-controller interface for connecting with the MoveIt planning pipeline.

Note 1: This package assumes that the user is using the default setup provided by the Neobotix. 

1. Start the robot bringup by running

	`roslaunch neo_mp_400 bringup.launch `

This is by default started automatically when the robot is switched on. 

2. Open the RViz with the pre-defined configuration. For this, please run:

	`roslaunch neo_mp_400 rviz_navigation.launch`

3. Once RViz is open, enable the Motors by pressing the "Enable" button that can be found in RViz.

4. Reference the robot, in order to reset any errors accumulated by the encoder due to unforeseen reasons such as gravity, manual movement etc. Now you need to Enable the motors once again.  

5. By default the robot arm is set to "Velocity Mode", Once the enable button is pressed, you will be able to use the individual sliders for the joints that can be found below the Enable button. 

6. If you want to operate the robot arm in the "Position Mode", you need to call a ROS-Service:

	**It is always necessary to perform referencing before switching to the position mode.**

	`rosservice call /StartPositionController "{}"`

7. In order to jog in position mode, please run the following command:
	
	`rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller `

Select the appropriate controller (there will be just one) and press the red button to enable the position jog. 

Feel free to move the robot using the position control. 

8. If you want to switch back to the "Velocity Mode", you need to call a ROS-Service:

	`rosservice call /StopPositionController "{}"`

Note 2: MoveIt package for this robot by default supports position interface, therefore it is a must to switch to position mode, while it is desired by the user to use MoveIt. 



