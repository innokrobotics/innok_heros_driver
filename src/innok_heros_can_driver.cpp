/* Copyright 2017 Innok Robotics GmbH */

#include <cmath>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>
#include <thread>

// Prototypes
void publishJoyMsg();
void publishOdomMsg();
void publishVoltageMsg();
void publishBatteryStateMsg();

// Variables
int can_socket;

double battery_voltage = 0;
double battery_percentage = 0;
double odom_orientation = 0;
double odom_pos_x = 0;
double odom_pos_y = 0;

double velocity = 0;
double rotational_velocity = 0;

double old_odom_orientation = 0;
double old_odom_pos_x = 0;
double old_odom_pos_y = 0;


rclcpp::Time old_tick;

uint8_t remote_buttons[8];
uint8_t remote_analog[8];

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisherOdom;
rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisherJoy;
rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisherBatteryState;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisherVoltage;

rclcpp::Node::SharedPtr nodeHandle;

bool can_running = false;

int can_open(const char * device)
{
    struct ifreq ifr;
    struct sockaddr_can addr;
    /* open socket */
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(can_socket < 0)
    {
        return (-1);
    }
    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, device);
    
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0)
    {
        return (-2);
    }
    addr.can_ifindex = ifr.ifr_ifindex;
    //fcntl(can_socket, F_SETFL, O_NONBLOCK);

    
    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        return (-3);
    }
    
    struct timeval tv;
    tv.tv_sec = 1;  // 1 second timeout 
    tv.tv_usec = 0;  // Not init'ing this can cause strange errors
    setsockopt(can_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval));
    
    return 0;
}

int can_send_frame(struct can_frame * frame)
{
    int retval;
    retval = write(can_socket, frame, sizeof(struct can_frame));
    if (retval != sizeof(struct can_frame))
    {
        return (-1);
    }
    else
    {
        return (0);
    }
}

void can_send_cmd_vel(double speed, double yawspeed, int modestate = 3)
{
    struct can_frame msg_send;
    
    msg_send.can_id = 200;
    msg_send.can_dlc = 5;
    
    int int_speed = speed * 100;
    int int_yaw = yawspeed * 100;
    msg_send.data[0] = int_speed;
    msg_send.data[1] = int_speed >> 8;
    msg_send.data[2] = int_yaw;
    msg_send.data[3] = int_yaw >> 8;
    msg_send.data[4] = modestate;
    
    can_send_frame(&msg_send);
}
    

void can_read_frames()
{
    struct can_frame frame_rd;
    while(read(can_socket, &frame_rd, sizeof(struct can_frame))>0)
    {
        // Remote Control Values
        if (frame_rd.can_id == 0x1E4)       
        {
            for (int i = 0; i < 8; i++)
                remote_analog[i] = frame_rd.data[i];
        }
        else if(frame_rd.can_id == 0x2E4)        // Switches
        {
            for (int i = 0; i < 8; i++)
                remote_buttons[i] = frame_rd.data[i];
            publishJoyMsg();
        }
        else if (frame_rd.can_id == 235)
        {
            battery_voltage = (frame_rd.data[1]|(frame_rd.data[2]<<8))*0.01;
            battery_percentage = frame_rd.data[0];
            publishVoltageMsg();
            publishBatteryStateMsg();
        }
        else if (frame_rd.can_id == 236)
        {
            odom_pos_x = (frame_rd.data[0]|(frame_rd.data[1]<<8)|(frame_rd.data[2]<<16)|(frame_rd.data[3]<<24))*0.001;
            odom_pos_y = (frame_rd.data[4]|(frame_rd.data[5]<<8)|(frame_rd.data[6]<<16)|(frame_rd.data[7]<<24))*0.001;
        }
        else if (frame_rd.can_id == 237)
        {
            odom_orientation = (frame_rd.data[0]|(frame_rd.data[1]<<8)|(frame_rd.data[2]<<16)|(frame_rd.data[3]<<24))*0.001;
            publishOdomMsg();
        }
    }
}
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br;

void publishOdomMsg()
{

    tf2::Quaternion q;
    q.setEuler(odom_orientation, 0, 0);
    geometry_msgs::msg::Quaternion odom_quaternion;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = rclcpp::Clock().now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = odom_pos_x;
    odom_msg.pose.pose.position.y = odom_pos_y;
    odom_msg.pose.pose.orientation = odom_quaternion;
    
    rclcpp::Time tick = rclcpp::Clock().now();

    rclcpp::Duration dt(tick - old_tick);
    
    double odom_covariance[] = {0.1,	0,	0,	0,	0,	0,
                                0,	0.1,	0,	0,	0,	0,
                                0,	0,	9999.0,	0,	0,	0,
                                0,	0,	0,	0.6,	0,	0,
                                0,	0,	0,	0,	0.6,	0,
                                0,	0,	0,	0,	0,	1.6};
    for(int i=0; i<36; i++)
    {
      odom_msg.pose.covariance[i] = 0;
      odom_msg.twist.covariance[i] = odom_covariance[i];
    }

    double dx = odom_pos_x - old_odom_pos_x;
    double dy = odom_pos_y - old_odom_pos_y;
    double dalpha = odom_orientation - old_odom_orientation;
    
    if (dalpha > M_PI)
        dalpha -= 2.0 * M_PI;
    if (dalpha < -M_PI)
        dalpha += 2.0 * M_PI;

    if (pow(dx, 2) + pow(dy, 2) <= 1.0) {
        velocity = sqrt(pow(dx, 2)+pow(dy, 2)) / dt.seconds();
        rotational_velocity = dalpha / dt.seconds();
    }

    odom_msg.twist.twist.linear.x = (dx * cos(-odom_orientation) - dy * sin(-odom_orientation)) / dt.seconds();
    odom_msg.twist.twist.linear.y = (dx * sin(-odom_orientation) + dy * cos(-odom_orientation)) / dt.seconds();
    odom_msg.twist.twist.angular.z = rotational_velocity;
        
    // Publish odometry message
    publisherOdom->publish(odom_msg);

    //update last values
    old_odom_orientation = odom_orientation;
    old_odom_pos_x = odom_pos_x;
    old_odom_pos_y = odom_pos_y;
    old_tick = tick;
    
    // Also publish tf if necessary
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = odom_msg.header.stamp;
    odom_trans.header.frame_id = odom_msg.header.frame_id;
    odom_trans.child_frame_id = odom_msg.child_frame_id;
    
    odom_trans.transform.translation.x = odom_pos_x;
    odom_trans.transform.translation.y = odom_pos_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quaternion;
    
    //tf_br->sendTransform(odom_trans);
}

void publishVoltageMsg()
{
    std_msgs::msg::Float32 voltage_msg;
    voltage_msg.data = battery_voltage;
    publisherVoltage->publish(voltage_msg);
}

/*
 * @author Sabrina Heerklotz
 * @date June 2018
 * 
 * publishes the battery state
 */
void publishBatteryStateMsg()
{
    sensor_msgs::msg::BatteryState state_msg;
    state_msg.voltage = battery_voltage;
    state_msg.current = NAN;
    state_msg.charge = NAN;
    state_msg.capacity = NAN;
    state_msg.design_capacity = NAN;
    state_msg.percentage = battery_percentage;
    state_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    state_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    state_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    state_msg.present = true;
    for (int i = 0; i < 13; i++)
        state_msg.cell_voltage.push_back(NAN);
    state_msg.location = "ROCTR";
    state_msg.serial_number = "";

    
    publisherBatteryState->publish(state_msg);
}

void publishJoyMsg()
{
    sensor_msgs::msg::Joy rc_msg;
    
    // analog axes
    for(int i=0; i<=8; i++)
    {
        rc_msg.axes.push_back( (remote_analog[i] - 127) / 127.0);
    }
    
    // buttons
    for(int i=0; i<=8; i++)
    {
        for(int bit=0; bit<=8; bit++)
        {
            if(remote_buttons[i] & (1 << bit))
                rc_msg.buttons.push_back(bool(true));
            else
                rc_msg.buttons.push_back(bool(false));
        }
    }
    publisherJoy->publish(rc_msg);
}

void cmdVelCallback(const geometry_msgs::msg::Twist& msg)
{
    RCLCPP_DEBUG(nodeHandle->get_logger(), "received cmd_vel s=%f y=%f", msg.linear.x, msg.angular.z);
    can_send_cmd_vel(msg.linear.x, msg.angular.z);
}

void can_task()
{
    can_running = true;
    while(can_running)
    {
        can_read_frames();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    nodeHandle = std::make_shared<rclcpp::Node>("innok_heros_can_driver");
    auto cmdVelSub = nodeHandle->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, cmdVelCallback);
    publisherOdom = nodeHandle->create_publisher<nav_msgs::msg::Odometry>("odom", 20);
    publisherVoltage = nodeHandle->create_publisher<std_msgs::msg::Float32>("battery_voltage", 1);
    publisherBatteryState = nodeHandle->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 1);
    publisherJoy = nodeHandle->create_publisher<sensor_msgs::msg::Joy>("remote_joy", 1);
    
    std::make_unique<tf2_ros::TransformBroadcaster>(nodeHandle);



    can_open("can0"); // TODO: parameter for can device	
    
    std::thread can_thread(can_task);

    rclcpp::spin(nodeHandle);
    rclcpp::shutdown();
    can_running = false;
}
