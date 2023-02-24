# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.

"""
Solution by:
Tawsiful Islam tawsiful@kth.se
Linus Lundvall llundv@kth.se
"""


import py_trees as pt, py_trees_ros as ptr, rospy
import actionlib
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import numpy as np
from numpy import linalg as LA

resetVariable = False
reset_call = False

class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):
        rospy.loginfo("Initialising counter behaviour.")

        # Counter
        self.i = 0
        self.n = n
        super(counter, self).__init__(name)    # Become a behaviour

    def update(self):
        self.i += 1
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS # succeed after count is done


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):
        rospy.loginfo("Initialising go behaviour.")

        # Action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # Command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular
	
        super(go, self).__init__(name) # become a behaviour

    def update(self):
        # Send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()
        return pt.common.Status.RUNNING # Tell the tree that behaviour is running

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and vice versa.
    """

    def __init__(self):
        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # Personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # Execution checker
        self.sent_goal = False
        self.finished = False

        super(tuckarm, self).__init__("Tuck arm!")

    def reset(self):
        self.sent_goal = False
        self.finished = False


    def update(self):
        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # Command to tuck arm if it's done already
        elif not self.sent_goal:
            # Send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True
            return pt.common.Status.RUNNING    # Tell the tree it's running

        # If successful
        elif self.play_motion_ac.get_result():
            # It's finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE    # If failed

        # If it's still trying :|
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raises the head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and vice versa.
    """

    def __init__(self, direction):
        rospy.loginfo("Initialising move head behaviour.")

        # Server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # Head movement direction; "down" or "up"
        self.direction = direction

        # Execution checker
        self.tried = False
        self.done = False

        super(movehead, self).__init__("Lower head!")    # Become a behaviour

    def reset(self):
        self.tried = False
        self.done = False

    def update(self):
        # Success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # Try if not tried
        elif not self.tried:
            # Command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # Tell the tree it's running
            return pt.common.Status.RUNNING

        # If succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # If failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # If still trying
        else:
            return pt.common.Status.RUNNING

class Localisation(pt.behaviour.Behaviour):
	
    """
    Localises the robot by using AMCL and particle cloud. 
    Contains a subscriber that listens and checks continously 
    if the particle cloud is converged.
    Returns success if the particle cloud converges,
    running whilst turning, and failure when the cloud doesn't converge
    and will retry locating itself.
    """

    def __init__(self,name,localisation_srv_nm,amcl_estim_top,cmd_vel_top,clear_cost_srv_nm):
        self.local_srv = rospy.ServiceProxy(localisation_srv_nm, Empty)

        self.localised = False
        self.call_srv = True
        self.amcl_estim_top = amcl_estim_top
        self.local_srv()    # Spreads out the particles
        self.clear_cost_srv = rospy.ServiceProxy(clear_cost_srv_nm,Empty)   #Helps clearing obstacles from global planning
        
        self.part_cloud_cov = None
        self.local_sub = rospy.Subscriber(amcl_estim_top, PoseWithCovarianceStamped, self.get_part_cloud)   # Allows us to get cov_mat continously

        self.cmd_vel_pub = rospy.Publisher(cmd_vel_top,Twist,queue_size=10)
        self.rot_count  = 0
        self.max_rot = 120
        self.move_msg = Twist()
        self.move_msg.angular.z = 0.5
        self.move_msg.linear.x = 0
        self.rate = rospy.Rate(10)
        print("created local")

        rospy.loginfo("Initialising Localisation behaviour.")
        super(Localisation, self).__init__(name)

    def get_part_cloud(self,part_cloud):
        self.part_cloud_cov = np.reshape(part_cloud.pose.covariance,(6,6))  # Extract the covariance matrix

    def initialise(self):
        if self.localised == False and self.call_srv == True:
            print("lets spread")
            self.local_srv()    # If we did not converge or do first time, we spread the particles
            self.call_srv = False   # Makes sure we don't spread the particles again until we have rotated completely
            self.clear_cost_srv()   # Clear the cost map to prepare for global planning navigation
	
    def update(self):
        if self.localised == False: # If we have not localised, spin once to not get matrix = none
            self.move_msg.angular.z = 0.5
            self.cmd_vel_pub.publish(self.move_msg)
            self.rate.sleep()
            self.rot_count += 1
        if part_cloud_converge(self.part_cloud_cov):    # If variance converges, we are localised
            self.localised = True
            self.call_srv = True    # Allows to run initialise next time its needed
            self.rot_count = 0
            return pt.common.Status.SUCCESS
        else:
            self.localised = False
            if self.rot_count >= self.max_rot:  # Rotate 2 rotations
                self.rot_count = 0
                self.call_srv = True
            if self.call_srv == True:
                return pt.common.Status.FAILURE # If we have rotated and not converged
            return pt.common.Status.RUNNING # Still rotating

def part_cloud_converge(cov_mat):
	tr_cov = np.trace(cov_mat)  # Trace to only get variance/error in position
	if tr_cov <0.045:	# Check the maximum convergence is less than 0.45, then it has converged
		return True
	else:
		return False

class Navigation(pt.behaviour.Behaviour):
	
    """
    Path planning is decided whether the cube will be picked up or placed down.
    Kidnapping will be checked during navigation with a subscriber.
    Returns success when reaching the goal, running when its navigating and moving,
    failure when it fails finding a path.
    """
	
    def __init__(self,name,goal_str,pick_pose_top, place_pose_top, move_base_nm,amcl_estim_top,clear_cost_srv_nm):
        self.pick_pose_top = pick_pose_top
        self.place_pose_top = place_pose_top
        self.moveBase_ac = SimpleActionClient(move_base_nm, MoveBaseAction) # Main component for navigation and controls
        self.goal_str = goal_str    # Allows to know what goal msg should be
        self.goal_msg = MoveBaseGoal()
        self.nav_success = None
        self.reached_goal = False
        self.sent_goal = False
        self.status = None
        self.clear_cost_srv = rospy.ServiceProxy(clear_cost_srv_nm,Empty)
        self.part_cloud_cov = None
        self.local_sub = rospy.Subscriber(amcl_estim_top, PoseWithCovarianceStamped, self.get_part_cloud)
        

        if self.goal_str == "pick":
            self.goal_msg.target_pose = rospy.wait_for_message(self.pick_pose_top,PoseStamped,5)
        elif self.goal_str == "place":
            self.goal_msg.target_pose = rospy.wait_for_message(self.place_pose_top,PoseStamped,5)
        else:
            rospy.loginfo("Not a valid goal") 

        rospy.loginfo("Initialising Navigation behaviour.")
        super(Navigation, self).__init__(name)

    def get_part_cloud(self,part_cloud):
        self.part_cloud_cov = np.reshape(part_cloud.pose.covariance,(6,6))

    def goal_cb(self,status,result):
        self.status = status

    def reset(self):
        self.nav_success = None
        self.reached_goal = False
        self.sent_goal = False
        self.status = None

    def update(self):
        if self.reached_goal == True:
            return pt.common.Status.SUCCESS

        elif not self.sent_goal:    # Decide what the goal msg should depending on target
            if self.goal_str == "pick":
                self.goal_msg.target_pose = rospy.wait_for_message(self.pick_pose_top,PoseStamped,5)
            elif self.goal_str == "place":
                self.goal_msg.target_pose = rospy.wait_for_message(self.place_pose_top,PoseStamped,5)
            self.moveBase_ac.send_goal(self.goal_msg,done_cb=self.goal_cb)
            self.sent_goal = True
            return pt.common.Status.RUNNING
        
        elif (self.status == actionlib.TerminalState.ABORTED) or (self.status == actionlib.TerminalState.PREEMPTED) or (not part_cloud_converge(self.part_cloud_cov)):
            self.reached_goal = False   # If we have failed to finish the navigation
            self.moveBase_ac.cancel_all_goals() # Make sure that no existing goals are left after we are done
            return pt.common.Status.FAILURE
        elif self.status == actionlib.TerminalState.SUCCEEDED:
            print("goal success")
            self.reached_goal = True
            self.moveBase_ac.cancel_all_goals()
            return pt.common.Status.SUCCESS
        
        else:
            return pt.common.Status.RUNNING

class RespawnCube(pt.behaviour.Behaviour):
	
    """
    When the robot doesn't detect the cube to be placed correctly
    the cube is respawn to the first table with a known pose.
    Returns success when cube respawns correctly, failure
    if not, and running when not determined.
    """

    def __init__(self, name):
        self.respawned = False
        self.tried = False
        self.done = False
        self.cube_pos_msg = ModelState()
        self.cube_pos_msg.model_name = 'aruco_cube'
        self.cube_pos_msg.pose.position.x = -1.130530
        self.cube_pos_msg.pose.position.y = -6.653650
        self.cube_pos_msg.pose.position.z = 0.86250
        self.cube_pos_msg.reference_frame = "map"
        rospy.wait_for_service('/gazebo/set_model_state')   # Service that respawns the cube
        self.respawn_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.loginfo("Initialising RespawnCube behaviour.")
        super(RespawnCube,self).__init__(name)

    def reset(self):
        self.done = False
        self.tried = False
        self.respawned = False

    def update(self):
        if self.done == True:
            return pt.common.Status.SUCCESS

        elif self.tried == False:
            self.respawn_req = self.respawn_srv(self.cube_pos_msg)  # Respawns the cube by sending the msg
            self.tried = True
            return pt.common.Status.RUNNING
        elif self.respawn_req.success == True:
            self.done = True
            rospy.loginfo("Respawning the cube")
            return pt.common.Status.SUCCESS
        elif self.respawn_req.success == False:
            self.done = True
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

class DetectCube(pt.behaviour.Behaviour):
	
    """
    Checks if the cube is detected by checking if a message 
    regarding the cube's position can be recieved from Aruco topic.
    If not detected, the reset function will be triggered.
    Returns success if the cube is found and failure if not found.
    """

    def __init__(self,name, aruco_pose_top):
        self.foundCube = None
        self.aruco_pose_top = aruco_pose_top

        rospy.loginfo("Initialising DetectCube behaviour.")
        super(DetectCube, self).__init__(name)

    def reset(self):
        self.foundCube = None
	
    def update(self):
        global reset_call
        
        if self.foundCube == True:
            return pt.common.Status.SUCCESS
        if self.foundCube == False:
            return pt.common.Status.FAILURE

        try:
            cube_pose = rospy.wait_for_message(self.aruco_pose_top,PoseStamped,timeout=10)  # See if we can get the position of the cube
            self.foundCube = True
        except:
            cube_pose = None
	
        if not cube_pose:
            self.foundCube = False
            reset_call = True   # Allows the reset behaviours to run
            rospy.loginfo("Failed to find the cube.")
            return pt.common.Status.FAILURE
        else:
            rospy.loginfo("Detected the cube.")
            return pt.common.Status.SUCCESS



class PickCube(pt.behaviour.Behaviour):
	
    """
    Picks the cube by calling to the pick-up service.
    Returns success if the picking succeeds, otherwise failure.
    """

    def __init__(self,name,pick_srv_nm):
        self.picked = False
        self.pick_srv = rospy.ServiceProxy(pick_srv_nm,SetBool)
        rospy.loginfo("Initialising PickCube behaviour.")
        super(PickCube, self).__init__(name)
    
    def reset(self):
        self.picked = False

    def update(self):
        
        if self.picked:
            return pt.common.Status.SUCCESS

        pick_request = self.pick_srv()  # Calling this service will pick the cube according to its postion in aruco topic

        if pick_request.success:
            rospy.loginfo("Cube is picked up.")
            self.picked = True
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo("Cube was not picked up.")
            return pt.common.Status.FAILURE

class PlaceCube(pt.behaviour.Behaviour):
	
    """
    Place the cube by calling to the place-down service.
    Returns success if the placing succeeds, otherwise failure.
    """

    def __init__(self,name,place_srv_nm):
        self.placed = False
        self.place_srv = rospy.ServiceProxy(place_srv_nm,SetBool)
        rospy.loginfo("Initialising PlaceCube behaviour.")
        super(PlaceCube, self).__init__(name)

    def reset(self):
        self.placed = False

    def update(self):
        if self.placed:
            return pt.common.Status.SUCCESS

        place_request = self.place_srv()
	
        if place_request.success:
            rospy.loginfo("Cube has been placed.")
            self.placed = True
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo("Cube was not placed.")
            return pt.common.Status.FAILURE

class reset(pt.behaviour.Behaviour):
	
    """
    Resets all conditional variable in all behaviours
    to be able to rerun same behaviours when robot needs to 
    pick up the cube again.
    Returns success after reseting all behaviours, if the
    reset function is not called, failure is returned.
    """

    def __init__(self,name,beh_list):
        self.beh_list = beh_list
        rospy.loginfo("Initialising Reset behaviour.")
        super(reset, self).__init__(name)

    def update(self):
        global reset_call

        if reset_call == True:
            for behav in self.beh_list:
                behav.reset()   # Goes through all created behaviours and reset variables that prevents them to be run again
            reset_call = False  # Makes sure this cannot be called again
            return pt.common.Status.SUCCESS # Allows to exit the reset sequence
        else:
            return pt.common.Status.FAILURE
