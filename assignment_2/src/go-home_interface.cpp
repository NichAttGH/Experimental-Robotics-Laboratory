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

// logging function
void logMessage(const string& msg) {
    ROS_INFO("%s", msg.c_str());
}

// Class to handle waypoints coordinates
class WaypointHandler {
public:
    WaypointHandler() {
        waypoints = {
            {"wp0", make_tuple(0.0, 1.0)},
            {"wp1", make_tuple(6.0, 2.0)},  
            {"wp2", make_tuple(7.0, -5.0)},
            {"wp3", make_tuple(-3.0, -8.0)},
            {"wp4", make_tuple(-7.0, 1.5)}  
        };
    }

    tuple<float, float> getCoordinates(const string& waypoint) const {
        return waypoints.at(waypoint);
    }

private:
    unordered_map<string, tuple<float, float>> waypoints;
};

namespace KCL_rosplan {
    ActionInterfaceExtended::ActionInterfaceExtended(ros::NodeHandle &node_handle) {}

    bool ActionInterfaceExtended::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& message) {
            
        logMessage("*** go-home concreteCallback ***");

        WaypointHandler waypoint_handler;
        auto coords = waypoint_handler.getCoordinates(message->parameters[2].value);
        
        char x_coord[8], y_coord[8];
        sprintf(x_coord, "x: %0.1f", get<0>(coords));
        sprintf(y_coord, "y: %0.1f", get<1>(coords));
        
        logMessage(x_coord);
        logMessage(y_coord);

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client("move_base", true);
        
        while (!move_base_client.waitForServer(ros::Duration(5.0))) {
            logMessage("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal move_goal;
        move_goal.target_pose.header.frame_id = "map";
        move_goal.target_pose.header.stamp = ros::Time::now();
        move_goal.target_pose.pose.position.x = get<0>(coords);
        move_goal.target_pose.pose.position.y = get<1>(coords);
        move_goal.target_pose.pose.orientation.w = 1.0;

        logMessage(" robot_move sending goal ");
        move_base_client.sendGoal(move_goal);

        logMessage(" waiting for result ");
        move_base_client.waitForResult();
        logMessage(" result done ");

        if (move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            logMessage("robot reached target position");
        } else {
            logMessage("robot missed target position for some reason");
        }
        
        logMessage("Action (" + message->name + ") performed: completed!");
        
        return true;
    }
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "gohome_interface", ros::init_options::AnonymousName);
    
    ros::NodeHandle node_handle("~");

    KCL_rosplan::ActionInterfaceExtended action_interface(node_handle);

    action_interface.runActionInterface();
    
    return 0;
}

