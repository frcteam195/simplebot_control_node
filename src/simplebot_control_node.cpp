
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <map>
#include <mutex>
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>


#include <rio_control_node/Motor_Control.h>

#define PORT     5809
#define MAXLINE 1024
#define HOSTNAME "10.0.2.99"


#pragma pack()
struct MotorUDPPacket
{
	uint32_t api_packet_id;
	float left_motor_val;
	float right_motor_val;
} _motorUDPPacket;

void *context;

ros::NodeHandle *node;

std::mutex motor_control_mutex;

void motorControlCallback(const rio_control_node::Motor_Control &msg)
{
	std::lock_guard<std::mutex> lock(motor_control_mutex);
	_motorUDPPacket.api_packet_id = 0x00000001;
	_motorUDPPacket.left_motor_val = 0;
	_motorUDPPacket.right_motor_val = 0;
	for (int i = 0; i < msg.motors.size(); i++)
	{
		switch (msg.motors[i].id)
		{
		case 0:
		{
			_motorUDPPacket.left_motor_val = msg.motors[i].output_value + msg.motors[i].arbitrary_feedforward;
		}
		break;
		case 1:
		{
			_motorUDPPacket.right_motor_val = msg.motors[i].output_value + msg.motors[i].arbitrary_feedforward;
		}
		break;
		default:
			break;
		}
	}
}

void motor_transmit_loop()
{
	char buffer[1500];
	memset(buffer, 0, 1500);

	sockaddr_in servaddr;
    int fd = socket(AF_INET,SOCK_DGRAM,0);
    if(fd<0){
        ROS_ERROR("cannot open socket");
    }
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(HOSTNAME);
    servaddr.sin_port = htons(PORT);

	ros::Rate rate(50);

	while (ros::ok())
	{
		{
			std::lock_guard<std::mutex> lock(motor_control_mutex);
			if (sendto(fd, &_motorUDPPacket, sizeof(_motorUDPPacket), 0, (sockaddr*)&servaddr, sizeof(servaddr)) < 0)
			{
				ROS_ERROR("Cannot send cksimplerobot control message");
			}
		}
		rate.sleep();
	}
	close(fd);
}

int main(int argc, char **argv)
{
	/**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
	ros::init(argc, argv, "listener");
	// GOOGLE_PROTOBUF_VERIFY_VERSION;

	ros::NodeHandle n;

	node = &n;
	std::thread motorSendThread(motor_transmit_loop);
	ros::Subscriber motorControl = node->subscribe("MotorControl", 100, motorControlCallback);

	ros::spin();

	return 0;
}