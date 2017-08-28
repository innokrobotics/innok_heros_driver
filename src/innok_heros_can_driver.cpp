/* Copyright 2017 Innok Robotics GmbH */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>


// Prototypes
void publishJoyMsg();
void publishOdomMsg();
void publishVoltageMsg();

// Variables
int can_socket;

double battery_voltage = 0;
double odom_orientation = 0;
double odom_pos_x = 0;
double odom_pos_y = 0;

uint8_t remote_buttons[8];
uint8_t remote_analog[8];

ros::Publisher publisherOdom;
ros::Publisher publisherJoy;
ros::Publisher publisherVoltage;

boost::mutex ros_mutex;
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

int can_send_cmd_vel(double speed, double yawspeed, int modestate = 3)
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
            publishVoltageMsg();
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

void publishOdomMsg()
{
    static tf::TransformBroadcaster tf_br;

    geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(odom_orientation);
    
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "/odom";
    odom_msg.child_frame_id = "/base_link";
    odom_msg.pose.pose.position.x = odom_pos_x;
    odom_msg.pose.pose.position.y = odom_pos_y;
    odom_msg.pose.pose.orientation = odom_quaternion;
    
    
    double odom_covariance[] = {0.1,	0,	0,	0,	0,	0,
                                0,	0.1,	0,	0,	0,	0,
                                0,	0,	9999.0,	0,	0,	0,
                                0,	0,	0,	0.6,	0,	0,
                                0,	0,	0,	0,	0.6,	0,
                                0,	0,	0,	0,	0,	0.1};
    for(int i=0; i<36; i++)
    {
      odom_msg.pose.covariance[i] = odom_covariance[i];
    }
        
    // Publish odometry message
    publisherOdom.publish(odom_msg);
    
    // Also publish tf if necessary
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = odom_msg.header.stamp;
    odom_trans.header.frame_id = odom_msg.header.frame_id;
    odom_trans.child_frame_id = odom_msg.child_frame_id;
    
    odom_trans.transform.translation.x = odom_pos_x;
    odom_trans.transform.translation.y = odom_pos_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quaternion;
    
    ros_mutex.lock();
    tf_br.sendTransform(odom_trans);
    ros_mutex.unlock();
}

void publishVoltageMsg()
{
    std_msgs::Float32 voltage_msg;
    voltage_msg.data = battery_voltage;
    ros_mutex.lock();
    publisherVoltage.publish(voltage_msg);
    ros_mutex.unlock();
}

void publishJoyMsg()
{
    sensor_msgs::Joy rc_msg;
    
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
    ros_mutex.lock();
    publisherJoy.publish(rc_msg);
    ros_mutex.unlock();
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_DEBUG("received cmd_vel s=%f y=%f", msg->linear.x, msg->angular.z);
    can_send_cmd_vel(msg->linear.x, msg->angular.z);
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
    ros::init(argc, argv, "innok_heros_can_driver");
    ros::NodeHandle n;
    ros::Subscriber Sub = n.subscribe("cmd_vel", 10, cmdVelCallback);
    
    publisherOdom = n.advertise<nav_msgs::Odometry>("odom", 20);
    publisherVoltage = n.advertise<std_msgs::Float32>("battery_voltage", 1);
    publisherJoy = n.advertise<sensor_msgs::Joy>("remote_joy", 1);
    
    can_open("can0"); // TODO: parameter for can device	
    
    boost::thread can_thread(can_task);
    
    while (ros::ok())
    {
        ros_mutex.lock();
        ros::spinOnce();
        ros_mutex.unlock();
    }
    can_running = false;
    can_thread.join();
    return 0;
}