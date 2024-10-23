//
// Created by Mu Shibo on 04/2024
//

#ifndef _MOTOR_CONFIG_H_
#define _MOTOR_CONFIG_H_

#include "ros/ros.h"
#include "stdint.h"
#include "can_msgs/Frame.h"
#include "motor_control/MotorFeedback.h"

#define Joint_Num 5
#define Belt_Reduction 1.24

#define KP_MAX 500
#define KP_MIN 0
#define KD_MAX 5
#define KD_MIN 0

#define P_MAX_6006 12.5
#define P_MIN_6006 -12.5
#define V_MAX_6006 45.0
#define V_MIN_6006 -45.0
#define T_MAX_6006 20.0
#define T_MIN_6006 -20.0

#define P_MAX_8006 12.5
#define P_MIN_8006 -12.5
#define V_MAX_8006 45.0
#define V_MIN_8006 -45.0
#define T_MAX_8006 20.0
#define T_MIN_8006 -20.0

typedef struct 
{
	uint16_t state;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_state_t;

typedef struct 
{
	float pos_d;
	float vel_d;
	float tor_d;
	float Kp;
	float Kd;
}motor_cmd_t;


class MotorControlSet
{
    public:
		MotorControlSet(ros::NodeHandle* node_handle, const std::string &can_rx, const std::string &can_tx, const std::string &leg);
		~MotorControlSet();

		// create subscriber
		ros::Subscriber can1_rev;
		// creat publisher
		ros::Publisher can1_pub;
		ros::Publisher leg_pub;

        void mit_ctrl_map(motor_cmd_t* motor_cmd, can_msgs::Frame* cansendata);
		void feedback_map(motor_state_t* motor_state, motor_control::MotorFeedback* joint_feedback);


		void can1_rx_Callback(can_msgs::Frame msg);

        can_msgs::Frame motor_enable(uint8_t ID);
        can_msgs::Frame motor_disable(uint8_t ID);
		can_msgs::Frame motor_zero(uint8_t ID);
		can_msgs::Frame motor_reset(uint8_t ID);

        motor_state_t motor_state[Joint_Num];
        motor_cmd_t motor_cmd[Joint_Num];
        can_msgs::Frame cansendata[Joint_Num];
        can_msgs::Frame canreceiveata[Joint_Num];

		motor_control::MotorFeedback joint_feedback;
};

#endif