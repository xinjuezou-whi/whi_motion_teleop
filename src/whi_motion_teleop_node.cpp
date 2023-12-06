/******************************************************************
vehicle motion control teleop node for ROS
main function

Features:
- send cmd_vel by keyboard
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2018-05-10: Initial version
2020-10-22: refactoring
2020-xx-xx: xxx
******************************************************************/
#include <stdio.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <whi_interfaces/WhiEng.h>
#include <whi_interfaces/WhiMotionState.h>

#include <string>
#include <thread>
#include <signal.h>

static const char* VERSION = "01.13.3";
static double linear_min = 0.01;
static double linear_max = 2.5;
static double angular_min = 0.1;
static double angular_max = 1.6;
static double step_linear = 0.01;
static double step_angular = 0.1;
static bool cal_initiated = false;
static std::shared_ptr<ros::Publisher> pub_twist = nullptr;
static std::shared_ptr<ros::Publisher> pub_eng = nullptr;
static geometry_msgs::Twist msg_twist;
static whi_interfaces::WhiEng msg_eng;
static struct termios old_tio;
static std::atomic_bool terminating = false;
static std::shared_ptr<std::thread> th_handler = nullptr;
static std::atomic_bool remote_mode = false;
static std::atomic_bool toggle_estop = false;
static std::atomic_bool toggle_collision = false;

void printInstruction(double Linear, double Angular)
{
	printf("\n");
	printf("           w - forward\n");
	printf("a - left   s - stop      d - right\n");
	printf("           x - backward\n");
	printf("\n");
	printf("q - quit\n");
	printf("\n");
	printf("linear %.2f, angular %.2f\n", Linear, Angular);
}

void subCallbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& MotionState)
{
	if (MotionState->state == whi_interfaces::WhiMotionState::STA_ESTOP)
	{
		msg_twist.linear.x = 0.0;
		msg_twist.angular.z = 0.0;
		pub_twist->publish(msg_twist);

		if (!toggle_estop.load())
		{
			printf("[warn] E-Stop detected\n");
			printf("[cmd] linear %.2f, angular %.2f\n", msg_twist.linear.x, msg_twist.angular.z);
		}
		toggle_estop.store(true);
	}
	else if (MotionState->state == whi_interfaces::WhiMotionState::STA_ESTOP_CLEAR)
	{
		toggle_estop.store(false);
	}

	if (MotionState->state == whi_interfaces::WhiMotionState::STA_CRITICAL_COLLISION)
	{
		msg_twist.linear.x = 0.0;
		msg_twist.angular.z = 0.0;
		pub_twist->publish(msg_twist);

		if (!toggle_collision.load())
		{
			printf("[warn] collision detected\n");
			printf("[cmd] linear %.2f, angular %.2f\n", msg_twist.linear.x, msg_twist.angular.z);
		}
		toggle_collision.store(true);
	}
	else if (MotionState->state == whi_interfaces::WhiMotionState::STA_CRITICAL_COLLISION_CLEAR)
	{
		toggle_collision.store(false);
	}

	if (MotionState->state == whi_interfaces::WhiMotionState::STA_REMOTE)
	{
		if (!remote_mode.load())
		{
			msg_twist.linear.x = 0.0;
			msg_twist.angular.z = 0.0;
			pub_twist->publish(msg_twist);

			printf("[cmd] linear %.2f, angular %.2f\n", msg_twist.linear.x, msg_twist.angular.z);
		}
		remote_mode.store(true);
	}
	else if (MotionState->state == whi_interfaces::WhiMotionState::STA_AUTO)
	{
		remote_mode.store(false);
	}
}

void userInput()
{
	while (!terminating.load())
	{
		int ch = getchar();
		if (toggle_estop.load())
		{
			printf("\nE-Stop detected, command is ignored\n");
			continue;
		}
		else if (toggle_collision.load())
		{
			printf("\ncollision detected, command is ignored\n");
			continue;
		}
		else if (remote_mode.load())
		{
			printf("\nvehicle is in remote control mode, command is ignored\n");
			continue;
		}

		switch (ch)
		{
		case 32: // s
		case 115: // space
			// stop
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				msg_twist.linear.x = 0.0;
				msg_twist.angular.z = 0.0;
				pub_twist->publish(msg_twist);

				printf("[cmd] linear %.2f, angular %.2f\n", msg_twist.linear.x, msg_twist.angular.z);
			}
			break;
		case 97: // a
			// left
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (fabs(msg_twist.angular.z) < 1e-4)
				{
					msg_twist.angular.z = angular_min;
				}
				else
				{
					msg_twist.angular.z += step_angular;
					if (msg_twist.angular.z > angular_max)
					{
						msg_twist.angular.z = angular_max;
					}
				}
				pub_twist->publish(msg_twist);

				printf("[cmd] linear %.2f, angular %.2f\n", msg_twist.linear.x, msg_twist.angular.z);
			}
			break;
		case 100: // d
			// right
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (fabs(msg_twist.angular.z) < 1e-4)
				{
					msg_twist.angular.z = -angular_min;
				}
				else
				{
					msg_twist.angular.z -= step_angular;
					if (msg_twist.angular.z < -angular_max)
					{
						msg_twist.angular.z = -angular_max;
					}
				}
				pub_twist->publish(msg_twist);

				printf("[cmd] linear %.2f, angular %.2f\n", msg_twist.linear.x, msg_twist.angular.z);
			}
			break;
		case 119: // w
			// forward
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (fabs(msg_twist.linear.x) < 1e-4)
				{
					msg_twist.linear.x = linear_min;
				}
				else
				{
					msg_twist.linear.x += step_linear;
					if (msg_twist.linear.x > linear_max)
					{
						msg_twist.linear.x = linear_max;
					}
				}
				pub_twist->publish(msg_twist);

				printf("[cmd] linear %.2f, angular %.2f\n", msg_twist.linear.x, msg_twist.angular.z);
			}
			break;
		case 120: // x
			// backward
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (fabs(msg_twist.linear.x) < 1e-4)
				{
					msg_twist.linear.x = -linear_min;
				}
				else
				{
					msg_twist.linear.x -= step_linear;
					if (msg_twist.linear.x < -linear_max)
					{
						msg_twist.linear.x = -linear_max;
					}
				}
				

				printf("[cmd] linear %.2f, angular %.2f\n", msg_twist.linear.x, msg_twist.angular.z);
			}
			break;
		case 48: // 0
			msg_eng.eng_flag = 0;
			cal_initiated = false;
			pub_eng->publish(msg_eng);

			printf("[engineering] eng all neutralized\n");
			break;
		case 51: // 3: print imu
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (msg_eng.eng_flag & 0b00000100)
				{
					printf("[engineering] printf imu off\n");
					msg_eng.eng_flag &= ~0b00000100;
					msg_eng.eng_flag |= 0b00000001;
				}
				else
				{
					printf("[engineering] printf imu\n");
					msg_eng.eng_flag &= ~0b00000001;
					msg_eng.eng_flag |= 0b00000100;
				}
				pub_eng->publish(msg_eng);
			}
			break;
		case 52: // 4: reset imu
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				msg_eng.eng_flag |= 0b00001000;
				pub_eng->publish(msg_eng);
				msg_eng.eng_flag &= ~0b00001000;

				printf("[engineering] reset imu\n");
			}
			break;
		case 53: // 5: print enc
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (msg_eng.eng_flag & 0b00010000)
				{
					printf("[engineering] print incoder off\n");
					msg_eng.eng_flag &= ~0b00010000;
					msg_eng.eng_flag |= 0b00000010;
				}
				else
				{
					printf("[engineering] print incoder\n");
					msg_eng.eng_flag &= ~0b00000010;
					msg_eng.eng_flag |= 0b00010000;
				}
				pub_eng->publish(msg_eng);
			}
			break;
		case 54: // 6: reset enc
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				msg_eng.eng_flag |= 0b00100000;
				pub_eng->publish(msg_eng);
				msg_eng.eng_flag &= ~0b00100000;

				printf("[engineering] reset encoder\n");
			}
			break;
		case 55: // 7: build lookup
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				msg_eng.eng_flag |= 0b01000000;
				pub_eng->publish(msg_eng);
				msg_eng.eng_flag &= ~0b01000000;

				printf("[engineering] build lookup\n");
			}
			break;
		case 56: // 8: clear lookup
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				msg_eng.eng_flag |= 0b10000000;
				pub_eng->publish(msg_eng);
				msg_eng.eng_flag &= ~0b10000000;

				printf("[engineering] clear built lookup\n");
			}
			break;
		case 57: // 9: diameter compensation calibration
			cal_initiated = true;

			printf("[engineering] start diameter compensation calibration, please specify direction:\n");
			printf("[engineering] + counter-clock, - clock\n");
			break;
		case 45: // -: clockwise
			if (cal_initiated)
			{
				msg_eng.eng_flag |= 0b10100000000;
				pub_eng->publish(msg_eng);
				msg_eng.eng_flag &= ~0b10100000000;

				cal_initiated = false;

				printf("[engineering] diameter compensation calibration: clockwise\n");
				break;
			}
		case 43: // +: counter-clockwise
			if (cal_initiated)
			{
				msg_eng.eng_flag |= 0b01100000000;
				pub_eng->publish(msg_eng);
				msg_eng.eng_flag &= ~0b01100000000;

				cal_initiated = false;

				printf("[engineering] diameter compensation calibration: counter-clockwise\n");
				break;
			}
		default:
			if (cal_initiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				printf("[warn] unrecognized command. using following commands:\n");
				printInstruction(msg_twist.linear.x, msg_twist.angular.z);
			}
			break;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void sigintHandler(int sig)
{
	// Do some custom action.
	// For example, publish a stop message to some other nodes.
	std::cout << "quiting......" << std::endl;

	/* restore the former settings */
	tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);

	msg_twist.linear.x = 0.0;
	msg_twist.angular.z = 0.0;
	pub_twist->publish(msg_twist);

	terminating.store(true);
	th_handler->join();
 
	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

int main(int argc, char** argv)
{
	std::string nodeName("whi_motion_teleop");
	ros::init(argc, argv, nodeName);

	ros::NodeHandle node;

	// Override the default ros sigint handler.
	// This must be set after the first NodeHandle is created.
	signal(SIGINT, sigintHandler);

	// params
	double frequency = 1.0;
	node.param(nodeName + "/command_frequency", frequency, 5.0);
	std::string stateTopic;
	node.param(nodeName + "/motion_state_topic", stateTopic, std::string(""));
	node.param(nodeName + "/linear/min", linear_min, 0.01);
	node.param(nodeName + "/linear/max", linear_max, 2.5);
	node.param(nodeName + "/linear/step", step_linear, 0.01);
	node.param(nodeName + "/angular/min", angular_min, 0.1);
	node.param(nodeName + "/angular/max", angular_max, 1.6);
	node.param(nodeName + "/angular/step", step_angular, 0.1);

	/* get the terminal settings for stdin */
	tcgetattr(STDIN_FILENO, &old_tio);
	/* we want to keep the old setting to restore them a the end */
	struct termios newTio = old_tio;
	/* disable canonical mode (buffered i/o) and local echo */
	newTio.c_lflag &= (~ICANON & ~ECHO);
	/* set the new settings immediately */
	tcsetattr(STDIN_FILENO, TCSANOW, &newTio);

	printf("\n");
	printf("TELEOP VERSION %s\n", VERSION);
	printf("Copyright © 2018-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n\n");
	printf("set linear range: %.2f to %.2f, step: %.2f\n", linear_min, linear_max, step_linear);
	printf("set angular range: %.2f to %.2f, step: %.2f\n", angular_min, angular_max, step_angular);
	printInstruction(0.0, 0.0);

	std::unique_ptr<ros::Subscriber> subMotionState = nullptr;
	if (!stateTopic.empty())
	{
		subMotionState = std::make_unique<ros::Subscriber>(
            node.subscribe<whi_interfaces::WhiMotionState>(stateTopic, 10, subCallbackMotionState));
	}

	pub_twist = std::make_shared<ros::Publisher>(node.advertise<geometry_msgs::Twist>("cmd_vel", 50));
	pub_eng = std::make_shared<ros::Publisher>(node.advertise<whi_interfaces::WhiEng>("eng", 50));

	// spawn a thread to fresh publication
	th_handler = std::make_shared<std::thread>(userInput);
	
	ros::Rate loopRate(frequency);
	while (node.ok())
	{
		if (!remote_mode.load())
		{
			pub_twist->publish(msg_twist);
		}

		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}