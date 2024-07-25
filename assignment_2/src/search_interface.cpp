#include "ros/ros.h"
#include <regex>
#include "../include/action_handler.h"
#include <unistd.h>
#include "assignment_2/RobotVision.h"
#include <geometry_msgs/Twist.h>

using std::string;
using std::regex;
using std::smatch;
using std::stoi;

int camera_id;
ros::Publisher velocity_publisher;

namespace KCL_rosplan {

	// logging function
	void logMessage(const string& message) {
		ROS_INFO("%s", message.c_str());
	}

	// Class to handle target ID extraction
	class TargetIDExtractor {
	public:
		int extractID(const string& parameter) {
			regex rx("[0-9]+");
			smatch match;
			regex_search(parameter, match, rx);
			return stoi(match[0]);
		}
	};

	ActionInterfaceExtended::ActionInterfaceExtended(ros::NodeHandle &node_handle) {}
	
	bool ActionInterfaceExtended::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& message) {
			
			logMessage("Entering concreteCallback function");

			TargetIDExtractor extractor;
			int target_id = extractor.extractID(message->parameters[2].value);
			
			geometry_msgs::Twist velocity_command;
			velocity_command.angular.z = 0.5;

			while (target_id != camera_id) {
				ROS_INFO("targetId: %d - cameraId: %d", target_id, camera_id);
				velocity_publisher.publish(velocity_command);
				ros::Duration(0.05).sleep();
			}

			velocity_command.angular.z = 0.0;
			velocity_publisher.publish(velocity_command);

			camera_id = 0;

			logMessage("Marker " + std::to_string(target_id) + " found. Action (" + message->name + ") completed!");
			return true;
		}
}

void visionCallback(const assignment_2::RobotVision::ConstPtr& message){
	camera_id = message->id;
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "search_interface_node", ros::init_options::AnonymousName);
	
	ros::NodeHandle node_handle;
	
	ros::Subscriber vision_subscriber = node_handle.subscribe("info_vision", 100, visionCallback);
	
	velocity_publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	
	KCL_rosplan::ActionInterfaceExtended action_interface(node_handle);
	
	action_interface.runActionInterface();
	
	return 0;
}

