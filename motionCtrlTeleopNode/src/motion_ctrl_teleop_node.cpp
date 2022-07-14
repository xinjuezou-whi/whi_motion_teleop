/******************************************************************
vehicle motion control teleop node for ROS
main function

Features:
- send cmd_vel by keyboard
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2019-05-10: Initial version
2020-10-22: refactoring
2020-xx-xx: xxx
******************************************************************/

#include <stdio.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <whi_interfaces/WhiEng.h>
#include <string>
#include <thread>
#include "../sdk/include/CUtility.h"
using namespace std;

static const char* VERSION = "01.11";
static double linearMin = 0.01;
static double linearMax = 2.5;
static double angularMin = 0.1;
static double angularMax = 1.6;
static double stepLinear = 0.01;
static double stepAngular = 0.1;
static int cmdRate = 200; // millisecond
static bool calInitiated = false;
static bool terminating = false;
static std::shared_ptr<std::thread> thHandler = nullptr;

void readConfig(const char* FileName)
{
	vector<vector<string>> configs;
	CUtility::readCsv(FileName, configs);
	for (vector<vector<string>>::const_iterator iter = configs.begin(); iter != configs.end(); ++iter)
	{
		if (iter->at(0) == "LIMITS_LINEAR")
		{
			linearMin = atof(iter->at(1).c_str());
			linearMax = atof(iter->at(2).c_str());
		}
		else if (iter->at(0) == "LIMITS_ANGULAR")
		{
			angularMin = atof(iter->at(1).c_str());
			angularMax = atof(iter->at(2).c_str());
		}
		else if (iter->at(0) == "STEP")
		{
			stepLinear = atof(iter->at(1).c_str());
			stepAngular = atof(iter->at(2).c_str());
		}
		else if (iter->at(0) == "CMD_FREQUENCY")
		{
			int frequency = atoi(iter->at(1).c_str());
			cmdRate = frequency == 0 ? 200 : 1000 / frequency;
		}
		else
		{
			// do nothing
		}
	}
}

void callbackFresher(std::shared_ptr<ros::Publisher> Pub, const geometry_msgs::Twist& Msg)
{
	while (!terminating)
	{
		Pub->publish(Msg);

		std::this_thread::sleep_for(std::chrono::milliseconds(cmdRate));
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dr_motion_ctrl_teleop");

	ros::NodeHandle node;

	ros::NodeHandle private_nh("~");
	string configFile;
	private_nh.param<string>("config_file", configFile, string("<your_workspace>/config/config.csv"));
	readConfig(configFile.c_str());

	struct termios old_tio, new_tio;

	/* get the terminal settings for stdin */
	tcgetattr(STDIN_FILENO, &old_tio);

	/* we want to keep the old setting to restore them a the end */
	new_tio = old_tio;

	/* disable canonical mode (buffered i/o) and local echo */
	new_tio.c_lflag &= (~ICANON & ~ECHO);

	/* set the new settings immediately */
	tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

	double targetLinearVel = 0.0;
	double targetAngularVel = 0.0;

	printf("\n");
	printf("TELEOP VERSION %s\n", VERSION);
	printf("Copyright © 2018-2023 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n\n");
	printf("set linear range: %.2f to %.2f, step: %.2f\n", linearMin, linearMax, stepLinear);
	printf("set angular range: %.2f to %.2f, step: %.2f\n", angularMin, angularMax, stepAngular);
	printf("\n");
	printf("           w - forward\n");
	printf("a - left   s - stop      d - right\n");
	printf("           x - backward\n");
	printf("\n");
	printf("q - quit\n");
	printf("\n");
	printf("linear %.2f, angular %.2f\n", targetLinearVel, targetAngularVel);

	std::shared_ptr<ros::Publisher> publisherCmd_ = std::make_shared<ros::Publisher>(node.advertise<geometry_msgs::Twist>("cmd_vel", 50));
	std::shared_ptr<ros::Publisher> publisherEng_ = std::make_shared<ros::Publisher>(node.advertise<whi_interfaces::WhiEng>("eng", 50));
	geometry_msgs::Twist messageCmd;
	whi_interfaces::WhiEng messageEng;

	// spawn a thread to fresh publication
	thHandler = std::make_shared<std::thread>(std::bind(&callbackFresher, publisherCmd_, std::ref(messageCmd)));
	
	ros::Rate loop_rate(10.0);
	while (!terminating && node.ok())
	{
		switch (getchar())
		{
		case 113: // q
			printf("quitting...\n");
			messageCmd.linear.x = 0.0;
			messageCmd.angular.z = 0.0;
			publisherCmd_->publish(messageCmd);

			terminating = true;
			break;
		case 32: // s
		case 115: // space
			// stop
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				messageCmd.linear.x = 0.0;
				messageCmd.angular.z = 0.0;
				publisherCmd_->publish(messageCmd);

				printf("[cmd] linear %.2f, angular %.2f\n", messageCmd.linear.x, messageCmd.angular.z);
			}
			break;
		case 97: // a
			// left
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (fabs(messageCmd.angular.z) < 1e-4)
				{
					messageCmd.angular.z = angularMin;
				}
				else
				{
					messageCmd.angular.z += stepAngular;
					if (messageCmd.angular.z > angularMax)
					{
						messageCmd.angular.z = angularMax;
					}
				}
				publisherCmd_->publish(messageCmd);

				printf("[cmd] linear %.2f, angular %.2f\n", messageCmd.linear.x, messageCmd.angular.z);
			}
			break;
		case 100: // d
			// right
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (fabs(messageCmd.angular.z) < 1e-4)
				{
					messageCmd.angular.z = -angularMin;
				}
				else
				{
					messageCmd.angular.z -= stepAngular;
					if (messageCmd.angular.z < -angularMax)
					{
						messageCmd.angular.z = -angularMax;
					}
				}
				publisherCmd_->publish(messageCmd);

				printf("[cmd] linear %.2f, angular %.2f\n", messageCmd.linear.x, messageCmd.angular.z);
			}
			break;
		case 119: // w
			// forward
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (fabs(messageCmd.linear.x) < 1e-4)
				{
					messageCmd.linear.x = linearMin;
				}
				else
				{
					messageCmd.linear.x += stepLinear;
					if (messageCmd.linear.x > linearMax)
					{
						messageCmd.linear.x = linearMax;
					}
				}
				publisherCmd_->publish(messageCmd);

				printf("[cmd] linear %.2f, angular %.2f\n", messageCmd.linear.x, messageCmd.angular.z);
			}
			break;
		case 120: // x
			// backward
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (fabs(messageCmd.linear.x) < 1e-4)
				{
					messageCmd.linear.x = -linearMin;
				}
				else
				{
					messageCmd.linear.x -= stepLinear;
					if (messageCmd.linear.x < -linearMax)
					{
						messageCmd.linear.x = -linearMax;
					}
				}
				

				printf("[cmd] linear %.2f, angular %.2f\n", messageCmd.linear.x, messageCmd.angular.z);
			}
			break;
		case 48: // 0
			messageEng.eng_flag = 0;
			calInitiated = false;
			publisherEng_->publish(messageEng);

			printf("[engineering] eng all neutralized\n");
			break;
		case 51: // 3: print imu
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (messageEng.eng_flag & 0b00000100)
				{
					printf("[engineering] printf imu off\n");
					messageEng.eng_flag &= ~0b00000100;
					messageEng.eng_flag |= 0b00000001;
				}
				else
				{
					printf("[engineering] printf imu\n");
					messageEng.eng_flag &= ~0b00000001;
					messageEng.eng_flag |= 0b00000100;
				}
				publisherEng_->publish(messageEng);
			}
			break;
		case 52: // 4: reset imu
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				messageEng.eng_flag |= 0b00001000;
				publisherEng_->publish(messageEng);
				messageEng.eng_flag &= ~0b00001000;

				printf("[engineering] reset imu\n");
			}
			break;
		case 53: // 5: print enc
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				if (messageEng.eng_flag & 0b00010000)
				{
					printf("[engineering] print incoder off\n");
					messageEng.eng_flag &= ~0b00010000;
					messageEng.eng_flag |= 0b00000010;
				}
				else
				{
					printf("[engineering] print incoder\n");
					messageEng.eng_flag &= ~0b00000010;
					messageEng.eng_flag |= 0b00010000;
				}
				publisherEng_->publish(messageEng);
			}
			break;
		case 54: // 6: reset enc
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				messageEng.eng_flag |= 0b00100000;
				publisherEng_->publish(messageEng);
				messageEng.eng_flag &= ~0b00100000;

				printf("[engineering] reset encoder\n");
			}
			break;
		case 55: // 7: build lookup
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				messageEng.eng_flag |= 0b01000000;
				publisherEng_->publish(messageEng);
				messageEng.eng_flag &= ~0b01000000;

				printf("[engineering] build lookup\n");
			}
			break;
		case 56: // 8: clear lookup
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				messageEng.eng_flag |= 0b10000000;
				publisherEng_->publish(messageEng);
				messageEng.eng_flag &= ~0b10000000;

				printf("[engineering] clear built lookup\n");
			}
			break;
		case 57: // 9: diameter compensation calibration
			calInitiated = true;

			printf("[engineering] start diameter compensation calibration, please specify direction:\n");
			printf("[engineering] + counter-clock, - clock\n");
			break;
		case 45: // -: clockwise
			if (calInitiated)
			{
				messageEng.eng_flag |= 0b10100000000;
				publisherEng_->publish(messageEng);
				messageEng.eng_flag &= ~0b10100000000;

				calInitiated = false;

				printf("[engineering] diameter compensation calibration: clockwise\n");
				break;
			}
		case 43: // +: counter-clockwise
			if (calInitiated)
			{
				messageEng.eng_flag |= 0b01100000000;
				publisherEng_->publish(messageEng);
				messageEng.eng_flag &= ~0b01100000000;

				calInitiated = false;

				printf("[engineering] diameter compensation calibration: counter-clockwise\n");
				break;
			}
		default:
			if (calInitiated)
			{
				printf("[engineering] please specify direction:\n");
				printf("[engineering] + counter-clock, - clock\n");
			}
			else
			{
				printf("[warn] unrecognized command. using following commands:\n");
				printf("\n");
				printf("           w - forward\n");
				printf("a - left   s - stop      d - right\n");
				printf("           x - backward\n");
				printf("\n");
				printf("q - quit\n");
				printf("\n");
				printf("linear %.2f, angular %.2f\n", messageCmd.linear.x, messageCmd.angular.z);
			}
			break;
		}

		loop_rate.sleep();
	}

	/* restore the former settings */
	tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);

	thHandler->join();

	return 0;
}
