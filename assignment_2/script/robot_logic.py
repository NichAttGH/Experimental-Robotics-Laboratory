#! /usr/bin/env python3

"""

.. module: robot_logic
   :platform unix
   :synopsis: Python module for calling services regarding rosplan.
   
.. moduleauthor:: 

This ROS node is used for calling services regarding rosplan.

Services:
**/rosplan_problem_interface/problem_generation_server_params**
**/rosplan_planner_interface/planning_server_params**
**/rosplan_parsing_interface/parse_plan_from_file**
**/rosplan_plan_dispatcher/dispatch_plan**
  

"""

import rospy
import os

from rosplan_dispatch_msgs.srv import ( 
        PlanningService, PlanningServiceRequest, 
        ParsingService, ParsingServiceRequest, 
        DispatchService, DispatchServiceRequest, 
        ProblemService, ProblemServiceRequest
    )

from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from assignment_2.msg import LogicStates

# global variables
package_path = ""
planner_path = ""
data_path = ""
current_state = LogicStates.PROBLEM_GENERATION

def handle_action_feedback(feedback: ActionFeedback):

    """
    Function for printing if the action has been completed.
    """
    
    if (feedback.status == ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE):
        rospy.loginfo("Action completed")
    elif (feedback.status == ActionFeedback.ACTION_FAILED):
        rospy.loginfo("Action failed")

def handle_action_dispatch(action: ActionDispatch):

    """
    Function for printing the action.
    """
    
    rospy.loginfo("Executing action: %s" % action.name)

def routine():
    """
    Function for calling all the needed services.
    """
    global current_state

    rate = rospy.Rate(10)
    
    while (not rospy.is_shutdown()):
        if current_state == LogicStates.PROBLEM_GENERATION:
            rospy.loginfo("Generating problem")

            # subscribe to the service to generate the problem
            problem_generation_client = rospy.ServiceProxy("/rosplan_problem_interface/problem_generation_server_params", ProblemService)

            # set problem generation request
            problem_generation_request = ProblemServiceRequest()
            problem_generation_request.problem_path = os.path.join(package_path, "pddl/generated_problem.pddl")
            problem_generation_request.problem_string_response = False

            # request problem generation
            problem_generation_response = problem_generation_client(problem_generation_request)

            current_state = LogicStates.WAITING_PROBLEM

        
        if current_state == LogicStates.WAITING_PROBLEM:
            if problem_generation_response.problem_generated:
                current_state = LogicStates.PLAN_GENERATION
            else:
                rospy.logerr("Error in generating the problem")
                current_state = LogicStates.ERROR


        if current_state == LogicStates.PLAN_GENERATION:
            rospy.loginfo("Generating plan")

            # subscribe to planner generator service
            plan_generation_client = rospy.ServiceProxy("/rosplan_planner_interface/planning_server_params", PlanningService)
            
            # set request for plan generation
            plan_generation_request = PlanningServiceRequest()
            plan_generation_request.use_problem_topic = True
            plan_generation_request.problem_path = os.path.join(package_path, "pddl/planner_problem.pddl")
            plan_generation_request.domain_path = os.path.join(package_path, "pddl/domain.pddl")
            plan_generation_request.data_path = os.path.join(package_path, data_path)
            plan_generation_request.planner_command = planner_path + " DOMAIN PROBLEM"
            
            # request plan generation
            plan_generation_response = plan_generation_client(plan_generation_request)

            current_state = LogicStates.WAITING_PLAN


        if current_state == LogicStates.WAITING_PLAN:
            if plan_generation_response.plan_found:
                rospy.loginfo("Plan found")
                current_state = LogicStates.PLAN_PARSING
            else:
                rospy.logerr("Error in generating the plan")
                current_state = LogicStates.ERROR


        if current_state == LogicStates.PLAN_PARSING:
            rospy.logdebug("parsing plan")
            # subscribe to parse service
            parser = rospy.ServiceProxy("/rosplan_parsing_interface/parse_plan_from_file", ParsingService)

            # set request for plan parsing
            plan_parsing_request = ParsingServiceRequest()
            # this file name is hard-coded in the rosplan_planning_system package
            plan_parsing_request.plan_path = os.path.join(package_path, data_path, "plan.pddl")
            
            # request plan parsing
            plan_parsing_response = parser(plan_parsing_request)

            current_state = LogicStates.WAITING_PARSING

        
        if current_state == LogicStates.WAITING_PARSING:
            if (plan_parsing_response.plan_parsed):
                rospy.logdebug("plan parsed successfully")
                current_state = LogicStates.DISPATCH_ACTION
            else:
                rospy.logerr("Error in parsing the plan")
                current_state = LogicStates.ERROR


        if current_state == LogicStates.DISPATCH_ACTION:
            rospy.logdebug("dispatching actions")
            # listen to action_dispatch
            rospy.Subscriber("/rosplan_plan_dispatcher/action_dispatch", ActionDispatch, handle_action_dispatch)

            # listen to action_feedback
            rospy.Subscriber("/rosplan_plan_dispatcher/action_feedback", ActionFeedback, handle_action_feedback)

            # subscribe to action feedback topic
            dispatcher = rospy.ServiceProxy("/rosplan_plan_dispatcher/dispatch_plan", DispatchService)

            # trigger dispatcher
            dispatch_action_request = DispatchServiceRequest()
            dispatch_action_response = dispatcher(dispatch_action_request)

            current_state = LogicStates.WAITING_REACHING_GOAL

        
        if current_state == LogicStates.WAITING_REACHING_GOAL:
            # success,goal_achieved
            if (dispatch_action_response.goal_achieved):
                rospy.loginfo_once("Goal achieved!")
            else:
                rospy.logerr("Error while dispatching the plan actions")
                current_state = LogicStates.ERROR

        if current_state == LogicStates.ERROR:
            rospy.logerr("error")

        rate.sleep()


def main():

    #global variable
    global package_path, planner_path, data_path, current_state

    # initialize node and wait for service
    rospy.init_node("robot_logic")

    rospy.wait_for_service("/rosplan_problem_interface/problem_generation_server_params")
    rospy.logdebug("problem generation server...service found")

    rospy.wait_for_service("/rosplan_planner_interface/planning_server_params")
    rospy.logdebug("planning server...service found")

    rospy.wait_for_service("/rosplan_parsing_interface/parse_plan_from_file")
    rospy.logdebug("parse plan from file...service found")

    rospy.wait_for_service("/rosplan_plan_dispatcher/dispatch_plan")
    rospy.logdebug("dispatch plan...service found")

    # get package and planner path
    package_path = rospy.get_param("pkg_path")
    planner_path = rospy.get_param("planner_path")
    data_path = rospy.get_param("data_path")

    # starting state
    current_state = LogicStates.PROBLEM_GENERATION

    routine()

    rospy.spin()


if __name__ == "__main__":
    main()
