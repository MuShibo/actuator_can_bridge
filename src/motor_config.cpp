//
// Created by Mu Shibo on 04/2024
//

#include "motor_control/motor_config.h"
#include "motor_control/math_ops.h"

MotorControlSet::MotorControlSet(ros::NodeHandle* node_handle, const std::string &can_rx, const std::string &can_tx, const std::string &leg){
    std::cout << leg << " is registering:" << std::endl;

    can1_rev = node_handle->subscribe(can_rx, 100, &MotorControlSet::can1_rx_Callback, this);
    can1_pub = node_handle->advertise<can_msgs::Frame>(can_tx, 100);
    leg_pub = node_handle->advertise<motor_control::MotorFeedback>("/m1_fd", 100);

    // enable the motor
    ros::Duration(0.5).sleep(); 
    for(int i = 0; i < Joint_Num; i++){
        can1_pub.publish(motor_enable(i+1));
        std::cout << "Joint motor " << i+1 << " of " << leg << " is enabled."<< std::endl;
    }

    joint_feedback.state.resize(Joint_Num);
    joint_feedback.pos.resize(Joint_Num);
    joint_feedback.vel.resize(Joint_Num);
    joint_feedback.tor.resize(Joint_Num);
    joint_feedback.Tmos.resize(Joint_Num);
    joint_feedback.Tcoil.resize(Joint_Num);
}

MotorControlSet::~MotorControlSet(){
    for(int i = 0; i < Joint_Num; i++){
        can1_pub.publish(motor_disable(i+1));
    }
    std::cout << "All joint motor have been disabled." << std::endl;
}

void MotorControlSet::mit_ctrl_map(motor_cmd_t* motor_cmd, can_msgs::Frame* cansendata)
{
    for(int i = 0; i < Joint_Num; i++){
        uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
        int id = i + 1;
        if(id == 1 || id == 3){
            pos_tmp = float_to_uint(motor_cmd[i].pos_d, P_MIN_8006, P_MAX_8006, 16);
            vel_tmp = float_to_uint(motor_cmd[i].vel_d, V_MIN_8006, V_MAX_8006, 12);
            kp_tmp = float_to_uint(motor_cmd[i].Kp, KP_MIN, KP_MAX, 12);
            kd_tmp = float_to_uint(motor_cmd[i].Kd, KD_MIN, KD_MAX, 12);
            tor_tmp = float_to_uint(motor_cmd[i].tor_d, T_MIN_8006, T_MAX_8006, 12);
        }
        if(id == 4){
            pos_tmp = float_to_uint(motor_cmd[i].pos_d * Belt_Reduction, P_MIN_8006, P_MAX_8006, 16);
            vel_tmp = float_to_uint(motor_cmd[i].vel_d * Belt_Reduction, V_MIN_8006, V_MAX_8006, 12);
            kp_tmp = float_to_uint(motor_cmd[i].Kp, KP_MIN, KP_MAX, 12);
            kd_tmp = float_to_uint(motor_cmd[i].Kd, KD_MIN, KD_MAX, 12);
            tor_tmp = float_to_uint(motor_cmd[i].tor_d / Belt_Reduction, T_MIN_8006, T_MAX_8006, 12);
        }
        else if (id == 2 || id == 5){
            pos_tmp = float_to_uint(motor_cmd[i].pos_d, P_MIN_6006, P_MAX_6006, 16);
            vel_tmp = float_to_uint(motor_cmd[i].vel_d, V_MIN_6006, V_MAX_6006, 12);
            kp_tmp = float_to_uint(motor_cmd[i].Kp, KP_MIN, KP_MAX, 12);
            kd_tmp = float_to_uint(motor_cmd[i].Kd, KD_MIN, KD_MAX, 12);
            tor_tmp = float_to_uint(motor_cmd[i].tor_d, T_MIN_6006, T_MAX_6006, 12);
        }

        cansendata[i].id = i + 1;
        cansendata[i].dlc = 0x08;
        cansendata[i].data[0] = (pos_tmp >> 8);
        cansendata[i].data[1] = (pos_tmp & 0xFF);
        cansendata[i].data[2] = ((vel_tmp >> 4) & 0xFF);
        cansendata[i].data[3] = ((((vel_tmp & 0xF) << 4) & 0xFF) | ((kp_tmp >> 8) & 0xFF));
        cansendata[i].data[4] = (kp_tmp & 0xFF);
        cansendata[i].data[5] = ((kd_tmp >> 4) & 0xFF);
        cansendata[i].data[6] = ((((kd_tmp & 0xF) << 4) & 0xFF) | ((tor_tmp >> 8) & 0xFF));
        cansendata[i].data[7] = (tor_tmp & 0xFF);
    }     
}

void MotorControlSet::feedback_map(motor_state_t* motor_state, motor_control::MotorFeedback* motor_feedback){
    // Not used.
}

void MotorControlSet::can1_rx_Callback(can_msgs::Frame msg)
{   
    int id = (uint16_t)(msg.data[0]) & 0x0F;
    
    motor_state[id-1].state = (uint16_t)(msg.data[0]) >> 4;

    uint16_t pos_hex = (uint16_t)(msg.data[2] | (msg.data[1] << 8));
    uint16_t vel_hex = (uint16_t)((msg.data[4] >> 4) | (msg.data[3] << 4));
    uint16_t t_hex = (uint16_t)((msg.data[5] | (msg.data[4] & 0x0F) << 8));;
    
    if(id == 1 || id == 3){
        motor_state[id-1].pos = uint_to_float(pos_hex, P_MAX_8006, P_MIN_8006, 16);
        motor_state[id-1].vel = uint_to_float(vel_hex, V_MAX_8006, V_MIN_8006, 12);
        motor_state[id-1].tor = uint_to_float(t_hex, T_MAX_8006, T_MIN_8006, 12);
        motor_state[id-1].Tmos = msg.data[6];
        motor_state[id-1].Tcoil = msg.data[7];
    }
    else if(id == 4){
        motor_state[id-1].pos = uint_to_float(pos_hex, P_MAX_8006, P_MIN_8006, 16) / Belt_Reduction;
        motor_state[id-1].vel = uint_to_float(vel_hex, V_MAX_8006, V_MIN_8006, 12) / Belt_Reduction;
        motor_state[id-1].tor = uint_to_float(t_hex, T_MAX_8006, T_MIN_8006, 12) * Belt_Reduction;
        motor_state[id-1].Tmos = msg.data[6];
        motor_state[id-1].Tcoil = msg.data[7];
    }
    else if (id == 2 || id == 5){
        motor_state[id-1].pos = uint_to_float(pos_hex, P_MAX_6006, P_MIN_6006, 16);
        motor_state[id-1].vel = uint_to_float(vel_hex, V_MAX_6006, V_MIN_6006, 12);
        motor_state[id-1].tor = uint_to_float(t_hex, T_MAX_6006, T_MIN_6006, 12);
        motor_state[id-1].Tmos = msg.data[6];
        motor_state[id-1].Tcoil = msg.data[7];
    }
    else{
        ROS_WARN("Control loop has undefined motors!");
    }
    
    if(id == 4){
    
    }
    
    joint_feedback.state[id-1] = motor_state[id-1].state;
    joint_feedback.pos[id-1] = motor_state[id-1].pos;
    joint_feedback.vel[id-1] = motor_state[id-1].vel;
    joint_feedback.tor[id-1] = motor_state[id-1].tor;
    joint_feedback.Tmos[id-1] = motor_state[id-1].Tmos;
    joint_feedback.Tcoil[id-1] = motor_state[id-1].Tcoil;
    leg_pub.publish(joint_feedback);
    // ROS_INFO("Received!");
}


can_msgs::Frame MotorControlSet::motor_enable(uint8_t ID)
{
    can_msgs::Frame csd;
    csd.id = ID;
    csd.dlc = 0x08;
    csd.data[0] = 0xFF;
    csd.data[1] = 0xFF;
    csd.data[2] = 0xFF;
    csd.data[3] = 0xFF;
    csd.data[4] = 0xFF;
    csd.data[5] = 0xFF;
    csd.data[6] = 0xFF;
    csd.data[7] = 0xFC;
    
    return csd;
}

can_msgs::Frame MotorControlSet::motor_disable(uint8_t ID)
{
    can_msgs::Frame csd;
    csd.id = ID;
    csd.dlc = 0x08;
    csd.data[0] = 0xFF;
    csd.data[1] = 0xFF;
    csd.data[2] = 0xFF;
    csd.data[3] = 0xFF;
    csd.data[4] = 0xFF;
    csd.data[5] = 0xFF;
    csd.data[6] = 0xFF;
    csd.data[7] = 0xFD;
    
    return csd;
}

can_msgs::Frame MotorControlSet::motor_zero(uint8_t ID)
{
    can_msgs::Frame csd;
    csd.id = ID;
    csd.dlc = 0x08;
    csd.data[0] = 0xFF;
    csd.data[1] = 0xFF;
    csd.data[2] = 0xFF;
    csd.data[3] = 0xFF;
    csd.data[4] = 0xFF;
    csd.data[5] = 0xFF;
    csd.data[6] = 0xFF;
    csd.data[7] = 0xFE;
    
    return csd;
}

can_msgs::Frame MotorControlSet::motor_reset(uint8_t ID)
{
    can_msgs::Frame csd;
    csd.id = ID;
    csd.dlc = 0x08;
    csd.data[0] = 0xFF;
    csd.data[1] = 0xFF;
    csd.data[2] = 0xFF;
    csd.data[3] = 0xFF;
    csd.data[4] = 0xFF;
    csd.data[5] = 0xFF;
    csd.data[6] = 0xFF;
    csd.data[7] = 0xFB;
    
    return csd;
}




