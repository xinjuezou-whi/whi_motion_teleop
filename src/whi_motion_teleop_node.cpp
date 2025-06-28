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
#include <unistd.h>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <whi_interfaces/msg/whi_eng.hpp>
#include <whi_interfaces/msg/whi_motion_state.hpp>
#include <whi_interfaces/msg/whi_rc_state.hpp>

#include <string>
#include <thread>
#include <signal.h>

static const char* VERSION = "02.15.2";
static double linear_min = 0.01;
static double linear_max = 2.5;
static double angular_min = 0.1;
static double angular_max = 1.6;
static double step_linear = 0.01;
static double step_angular = 0.1;
static bool cal_initiated = false;
static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist;
static rclcpp::Publisher<whi_interfaces::msg::WhiEng>::SharedPtr pub_eng;
static rclcpp::Publisher<whi_interfaces::msg::WhiRcState>::SharedPtr pub_rc_state;
static geometry_msgs::msg::Twist msg_twist;
static whi_interfaces::msg::WhiEng msg_eng;
static struct termios old_tio;
static std::atomic_bool terminating = false;
static std::shared_ptr<std::thread> th_handler = nullptr;
static std::atomic_bool remote_mode = false;
static std::atomic_bool toggle_estop = false;
static std::atomic_bool toggle_collision = false;
static bool sw_estopped = false;

void printInstruction(double Linear, double Angular)
{
	std::cout << "\n" << std::endl;
	std::cout << "           w - forward" << std::endl;
	std::cout << "a - left   s - stop      d - right" << std::endl;
	std::cout << "           x - backward\n" << std::endl;
	std::cout << "c - clear AMR error\n" << std::endl;
	std::cout << "linear: " << Linear << " angular: " << Angular << std::endl;
}

void subCallbackMotionState(const whi_interfaces::msg::WhiMotionState::SharedPtr MotionState)
{
	if (MotionState->state == whi_interfaces::msg::WhiMotionState::STA_ESTOP)
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
	else if (MotionState->state == whi_interfaces::msg::WhiMotionState::STA_STANDBY)
	{
		toggle_estop.store(false);
	}

	if (MotionState->state == whi_interfaces::msg::WhiMotionState::STA_CRITICAL_COLLISION)
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
	else if (MotionState->state == whi_interfaces::msg::WhiMotionState::STA_CRITICAL_COLLISION_CLEAR)
	{
		toggle_collision.store(false);
	}
}

void subCallbackSwEstop(const std_msgs::msg::Bool::SharedPtr Msg)
{
	sw_estopped = Msg->data;
}

void subCallbackRcState(const whi_interfaces::msg::WhiRcState::SharedPtr RcState)
{
	if (RcState->state == whi_interfaces::msg::WhiRcState::STA_REMOTE)
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
	else if (RcState->state == whi_interfaces::msg::WhiRcState::STA_AUTO)
	{
		remote_mode.store(false);
	}
}

void userInput()
{
	while (!terminating.load())
	{
		int ch = getchar();
		if (ch != 115) // ignore stop command
		{
			if (toggle_estop.load() || sw_estopped)
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
				if (fabs(msg_twist.angular.z) < 1e-5)
				{
					msg_twist.angular.z = angular_min;
				}
				else
				{
					msg_twist.angular.z += step_angular;
				}

				if (msg_twist.angular.z > angular_max)
				{
					msg_twist.angular.z = angular_max;
				}
				if (fabs(msg_twist.angular.z) < angular_min)
				{
					msg_twist.angular.z = 0.0;
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
				if (fabs(msg_twist.angular.z) < 1e-5)
				{
					msg_twist.angular.z = -angular_min;
				}
				else
				{
					msg_twist.angular.z -= step_angular;
				}

				if (msg_twist.angular.z < -angular_max)
				{
					msg_twist.angular.z = -angular_max;
				}
				if (fabs(msg_twist.angular.z) < angular_min)
				{
					msg_twist.angular.z = 0.0;
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
				if (fabs(msg_twist.linear.x) < 1e-5)
				{
					msg_twist.linear.x = linear_min;
				}
				else
				{
					msg_twist.linear.x += step_linear;
				}
				
				if (msg_twist.linear.x > linear_max)
				{
					msg_twist.linear.x = linear_max;
				}
				if (fabs(msg_twist.linear.x) < linear_min)
				{
					msg_twist.linear.x = 0.0;
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
				if (fabs(msg_twist.linear.x) < 1e-5)
				{
					msg_twist.linear.x = -linear_min;
				}
				else
				{
					msg_twist.linear.x -= step_linear;
				}

				if (msg_twist.linear.x < -linear_max)
				{
					msg_twist.linear.x = -linear_max;
				}
				if (fabs(msg_twist.linear.x) < linear_min)
				{
					msg_twist.linear.x = 0.0;
				}
				
				printf("[cmd] linear %.2f, angular %.2f\n", msg_twist.linear.x, msg_twist.angular.z);
			}
			break;
		case 99: // c
			{
				whi_interfaces::msg::WhiRcState msgState;
                msgState.state = whi_interfaces::msg::WhiRcState::STA_CLEAR_FAULT;
                pub_rc_state->publish(msgState);
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
	rclcpp::shutdown();
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<rclcpp::Node>("whi_motion_teleop");

	// Override the default ros sigint handler.
	// This must be set after the first Node is created.
	signal(SIGINT, sigintHandler);

	// params
	double frequency = 1.0;
	node->declare_parameter("command_frequency", 5.0);
	node->get_parameter("command_frequency", frequency);
	std::string stateTopic, swEstopTopic, rcStateTopic;
	node->declare_parameter("motion_state_topic", std::string("motion_state"));
	node->get_parameter("motion_state_topic", stateTopic);
	node->declare_parameter("sw_estop_topic", std::string("estop"));
	node->get_parameter("sw_estop_topic", swEstopTopic);
	node->declare_parameter("rc_state_topic", std::string("rc_state"));
	node->get_parameter("rc_state_topic", rcStateTopic);
	node->declare_parameter("linear.min", 0.01);
	node->get_parameter("linear.min", linear_min);
	node->declare_parameter("linear.max", 2.5);
	node->get_parameter("linear.max", linear_max);
	node->declare_parameter("linear.step", 0.01);
	node->get_parameter("linear.step", step_linear);
	node->declare_parameter("angular.min", 0.1);
	node->get_parameter("angular.min", angular_min);
	node->declare_parameter("angular.max", 1.6);
	node->get_parameter("angular.max", angular_max);
	node->declare_parameter("angular.step", 0.1);
	node->get_parameter("angular.step", step_angular);
	linear_min = abs(linear_min);
	linear_max = abs(linear_max);
	step_linear = abs(step_linear);
	angular_min = abs(angular_min);
	angular_max = abs(angular_max);
	step_angular = abs(step_angular);

	/* get the terminal settings for stdin */
	tcgetattr(STDIN_FILENO, &old_tio);
	/* we want to keep the old setting to restore them a the end */
	struct termios newTio = old_tio;
	/* disable canonical mode (buffered i/o) and local echo */
	newTio.c_lflag &= (~ICANON & ~ECHO);
	/* set the new settings immediately */
	tcsetattr(STDIN_FILENO, TCSANOW, &newTio);

	std::cout << "\nTELEOP VERSION " << VERSION << std::endl;
	std::cout << "Copyright © 2018-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
	std::cout << "set linear range: [" << linear_min << ", " << linear_max << "], step: " << step_linear << std::endl;
	std::cout << "set angular range: [" << angular_min << ", " << angular_max << "], step: " << step_angular << std::endl;
	printInstruction(0.0, 0.0);

	rclcpp::Subscription<whi_interfaces::msg::WhiMotionState>::SharedPtr subMotionState;
	if (!stateTopic.empty())
	{
      	subMotionState = node->create_subscription<whi_interfaces::msg::WhiMotionState>(
      		stateTopic, 10, subCallbackMotionState);
	}
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subSwEstop;
	if (!swEstopTopic.empty())
	{
      	subSwEstop = node->create_subscription<std_msgs::msg::Bool>(
      		swEstopTopic, 10, subCallbackSwEstop);
	}
	rclcpp::Subscription<whi_interfaces::msg::WhiRcState>::SharedPtr subRcState;
	if (!rcStateTopic.empty())
	{
      	subRcState = node->create_subscription<whi_interfaces::msg::WhiRcState>(
      		rcStateTopic, 10, subCallbackRcState);
	}

	pub_twist = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 50);
	pub_eng = node->create_publisher<whi_interfaces::msg::WhiEng>("eng", 50);
	pub_rc_state = node->create_publisher<whi_interfaces::msg::WhiRcState>(rcStateTopic, 50);

	// spawn a thread to fresh publication
	th_handler = std::make_shared<std::thread>(userInput);

	auto loopPub = [&]()
	{
		if (!remote_mode.load())
		{
			pub_twist->publish(msg_twist);
		}
	};

	auto period = std::chrono::duration<double>(1.0 / frequency);
	auto timer = node->create_wall_timer(period, loopPub);

	rclcpp::spin(node);

	return 0;
}
