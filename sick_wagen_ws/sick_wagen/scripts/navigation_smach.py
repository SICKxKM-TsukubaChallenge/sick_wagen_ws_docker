#!/usr/bin/env python3
import rospy
import actionlib
import csv
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import smach
import smach_ros
from std_srvs.srv import Empty, EmptyResponse
import time

# WaypointをCSVファイルから読み込む関数
def load_waypoints_from_csv(filename):
	print("load_waypoints_from_csv()")
	waypoints = []
	with open(filename, 'r') as f:
		reader = csv.reader(f)
		for row in reader:
			pose = list(map(float, row[:7]))
			status = int(row[7])
			waypoints.append((pose, status))
	return waypoints

class WaypointManager:
	def __init__(self, filename):
		self.waypoints = load_waypoints_from_csv(filename)
		
	def get_next_waypoint(self):
		return self.waypoints.pop(0) if self.waypoints else None

class GoToWaypoint(smach.State):
	def __init__(self, waypoints):
		smach.State.__init__(self, outcomes=['next_waypoint', 'stop_required', 'signal_required', 'error', 'success'],  # Add a 'success' outcome
								output_keys=['waypoint_out'])
		self.waypoints = waypoints
		self.index = 0
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()

	def execute(self, userdata):
		if self.index < len(self.waypoints):
			pose, status = self.waypoints[self.index]
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z = pose[:3]
			goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = pose[3:7]
			self.client.send_goal(goal)

			self.client.wait_for_result()
			result = self.client.get_state()
			time.sleep(0.5)
			if result == actionlib.GoalStatus.SUCCEEDED:
				# Check the type of waypoint (0: normal, 1: stop, 2: signal)
				if status == 0:
					self.index += 1
					print("Next")
					return 'next_waypoint'
				elif status == 1:
					self.index += 1
					print("Stop location")
					return 'stop_required'
				elif status == 2:
					self.index += 1
					return 'signal_required'
			else:
				rospy.loginfo("Failed to move to waypoint. Retrying...")
				return 'error'
		else:
			return 'success'  # Return 'success' if all waypoints are processed
		
class TemporaryStop(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['done'])
		self.trigger_received = False
		self.srv = rospy.Service('restart', Empty, self.handle_restart)

	def handle_restart(self, req):
		rospy.loginfo("Restart service called. Robot will continue.")
		self.trigger_received = True
		return EmptyResponse()

	def execute(self, userdata):
		rospy.loginfo("Robot is temporarily stopped. Waiting for external trigger to continue...")
		rate = rospy.Rate(10)  # 10Hz
		while not rospy.is_shutdown():
			if self.trigger_received:
				self.trigger_received = False  # Reset the trigger flag for next time
				return 'done'
			rate.sleep()

class WaitForSignal(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['done'])

	def execute(self, userdata):
		# ここで信号待ちのロジックを実装...
		return 'done'

def main():
	print("Navigation")
	ROS_WAYPOINT_FILE = os.getenv('ROS_WAYPOINT_FILE')
	if ROS_WAYPOINT_FILE is None:
		print(f"WARNING Environment variable 'ROS_WAYPOINT_FILE' not found")
		rospy.signal_shutdown()

	waypoints = load_waypoints_from_csv(ROS_WAYPOINT_FILE)
	print("finish load_waypoint")

	rospy.init_node('navigation_smach')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['success', 'failure'])    
	# Open the container

	with sm:
		smach.StateMachine.add('TEMPORARY_STOP', TemporaryStop(),
						transitions={'done': 'GOTO_WAYPOINT'})
		smach.StateMachine.add('GOTO_WAYPOINT', GoToWaypoint(waypoints),
							transitions={'next_waypoint': 'GOTO_WAYPOINT',
											'stop_required': 'TEMPORARY_STOP',
											'signal_required': 'WAIT_FOR_SIGNAL',
											'error': 'GOTO_WAYPOINT'})
		smach.StateMachine.add('WAIT_FOR_SIGNAL', WaitForSignal(),
							   transitions={'done': 'GOTO_WAYPOINT'})
	print("finish StateMachine setting")
	
	sis = smach_ros.IntrospectionServer('navigation_smach', sm, '/NAVIGATION_SMACH')
	sis.start()

	# Execute SMACH plan
	print("Start Navigation!!")
	outcome = sm.execute()

	rospy.spin()
	sis.stop()
	print("End of navigation")

if __name__ == '__main__':
	main()