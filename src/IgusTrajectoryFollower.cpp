#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointJog.h>
#include <controller_manager/controller_manager.h>

class RLDP5  : public hardware_interface::RobotHW {

public:
  RLDP5() 
 { 

   // connect and register the joint state interface
	
    m_JointJogPublisher = m_node_handle.advertise<control_msgs::JointJog>("/JointJog", 50);
		m_JointStateSubscriber = m_node_handle.subscribe("joint_states", 10, &RLDP5::joint_state_callback, this);

   hardware_interface::JointStateHandle state_handle_a("arm_joint", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("joint1", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_b);

   hardware_interface::JointStateHandle state_handle_c("joint2", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_c);

   hardware_interface::JointStateHandle state_handle_d("joint3", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(state_handle_d);

   hardware_interface::JointStateHandle state_handle_e("joint4", &pos[4], &vel[4], &eff[4]);
   jnt_state_interface.registerHandle(state_handle_e);

   hardware_interface::JointStateHandle state_handle_f("joint5", &pos[5], &vel[5], &eff[5]);
   jnt_state_interface.registerHandle(state_handle_f);

   hardware_interface::JointStateHandle state_handle_g("joint6", &pos[6], &vel[6], &eff[6]);
   jnt_state_interface.registerHandle(state_handle_g);

   registerInterface(&jnt_state_interface);

   // connect and register the joint velocity interface
   hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("arm_joint"), &cmd[0]);
   jnt_vel_interface.registerHandle(vel_handle_a);

   hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("joint1"), &cmd[1]);
   jnt_vel_interface.registerHandle(vel_handle_b);

	hardware_interface::JointHandle vel_handle_c(jnt_state_interface.getHandle("joint2"), &cmd[2]);
   jnt_vel_interface.registerHandle(vel_handle_c);

   hardware_interface::JointHandle vel_handle_d(jnt_state_interface.getHandle("joint3"), &cmd[3]);
   jnt_vel_interface.registerHandle(vel_handle_d);

   hardware_interface::JointHandle vel_handle_e(jnt_state_interface.getHandle("joint4"), &cmd[4]);
   jnt_vel_interface.registerHandle(vel_handle_e);

   hardware_interface::JointHandle vel_handle_f(jnt_state_interface.getHandle("joint5"), &cmd[5]);
   jnt_vel_interface.registerHandle(vel_handle_f);

	hardware_interface::JointHandle vel_handle_g(jnt_state_interface.getHandle("joint6"), &cmd[6]);
   jnt_vel_interface.registerHandle(vel_handle_g);

   registerInterface(&jnt_vel_interface);
	
	}
	void write()
{
	control_msgs::JointJog msg;
	msg.header.stamp=ros::Time::now();
	msg.joint_names.push_back("arm_joint");
	msg.joint_names.push_back("joint1");
	msg.joint_names.push_back("joint2");
	msg.joint_names.push_back("joint3");
	msg.joint_names.push_back("joint4");
	msg.joint_names.push_back("joint5");
	msg.joint_names.push_back("joint6");
	msg.velocities.push_back(cmd[0]);
	msg.velocities.push_back(cmd[1]);
	msg.velocities.push_back(cmd[2]);
	msg.velocities.push_back(cmd[3]);
	msg.velocities.push_back(cmd[4]);
	msg.velocities.push_back(cmd[5]);
	msg.velocities.push_back(cmd[6]);
	m_JointJogPublisher.publish(msg);
}
	private:
		ros::NodeHandle m_node_handle;
		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::VelocityJointInterface jnt_vel_interface;
		double cmd[6];
		double pos[6];
		double vel[6];
		double eff[6];
    
    ros::Publisher m_JointJogPublisher;
		ros::Subscriber m_JointStateSubscriber;
    


void joint_state_callback(const sensor_msgs::JointState& joint_state)
{
	for(auto i = 0; i<=6; i++) {
		pos[i] = joint_state.position[i];
		vel[i] = joint_state.velocity[i];
		eff[i] = joint_state.effort[i];
	}
}



};



int main(int argc, char** argv) {
	ros::init(argc, argv, "igus_trajectory_follower");

	RLDP5 R1;
	

	ros::Duration period(0.03);	
	ros::Rate rate(30.0);


	controller_manager::ControllerManager cm(&R1);
	while (true)
  	{
		// Read the joints
		ros::spinOnce();
		rate.sleep();
    ros::Time now = ros::Time::now();
		// Update
		cm.update(now, period);

		// Write
		R1.write();
		
	}

	return 0;
}