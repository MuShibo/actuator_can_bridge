//
// Created by Mu Shibo on 04/2024
//

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "can_msgs/Frame.h"
#include "motor_control/motor_config.h"
#include "motor_control/MotorFeedback.h"
#include "motor_control/math_ops.h"

#include "stdint.h"
#include "math.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_control_test");
    ros::NodeHandle nm;
    ros::Rate loop_rate(200);

    std::string left_leg = "left_leg";
    MotorControlSet* _left_controller = new MotorControlSet(&nm, "can1_rx", "can1_tx", left_leg);
    
    ros::Duration(0.5).sleep(); 
    
    while (ros::ok())
    {   
        // Danger: The motor command must be inited. Otherwise the motor will lose control. 
        _left_controller->motor_cmd[0].pos_d = 0;
        _left_controller->motor_cmd[0].vel_d = 3;
        _left_controller->motor_cmd[0].Kp = 0;
        _left_controller->motor_cmd[0].Kd = 1;
        _left_controller->motor_cmd[0].tor_d = 0;

        _left_controller->mit_ctrl_map(_left_controller->motor_cmd, _left_controller->cansendata);

        for(int i = 0; i < Joint_Num; i++){
            _left_controller->can1_pub.publish(_left_controller->cansendata[i]);
            // ROS_INFO("Command of motor %d has sent success!", i+1);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Disable motor will not excute. Ros node has died.
    delete _left_controller; 

    ros::shutdown();
    return 0;
}

