#include "../include/action_handler.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <cstring>
#include <unordered_map>
#include <tuple>

using std::string;
using std::unordered_map;
using std::tuple;
using std::make_tuple;
using std::get;

namespace KCL_rosplan {

	// logging function
	void logMessage(const string& message) {
		ROS_INFO("%s", message.c_str());
	}

	// Class to handle waypoints coordinates
	class WaypointManager {
	public:
		WaypointManager() {
			waypoints = {
				{"wp0", make_tuple(0.0, 1.0)},
				{"wp1", make_tuple(6.0, 2.0)},	
				{"wp2", make_tuple(7.0, -5.0)},
				{"wp3", make_tuple(-3.0, -8.0)},
				{"wp4", make_tuple(-7.0, 1.5)} // 
			};
		}

		tuple<float, float> getCoordinates(const string& waypoint) const {
			return waypoints.at(waypoint);
		}

	private:
		unordered_map<string, tuple<float, float>> waypoints;
	};

	ActionInterfaceExtended::ActionInterfaceExtended(ros::NodeHandle &node_handle) {}
	
	bool ActionInterfaceExtended::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& message) {
			
			logMessage("Entering concreteCallback function");

			WaypointManager waypoint_manager;
			auto target_coords = waypoint_manager.getCoordinates(message->parameters[2].value);

			actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client("move_base", true);
			
			while (!action_client.waitForServer(ros::Duration(5.0))) {
				logMessage("Awaiting move_base action server");
			}

			move_base_msgs::MoveBaseGoal move_goal;
			move_goal.target_pose.header.frame_id = "map";
			move_goal.target_pose.header.stamp = ros::Time::now();
			move_goal.target_pose.pose.position.x = get<0>(target_coords);
			move_goal.target_pose.pose.position.y = get<1>(target_coords);
			move_goal.target_pose.pose.orientation.w = 1.0;

			logMessage("Sending goal to move_base");
			action_client.sendGoal(move_goal);

			logMessage("Waiting for the action result");
			action_client.waitForResult();
			logMessage("Result received");

			if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				logMessage("Target position reached");
			} else {
				logMessage("Failed to reach target position");
			}

			logMessage("Action (" + message->name + ") completed!");
			
			return true;
		}
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "goto_interface_node", ros::init_options::AnonymousName);
	
	ros::NodeHandle node_handle("~");

	KCL_rosplan::ActionInterfaceExtended action_interface(node_handle);
	
	action_interface.runActionInterface();
	
	return 0;
}

