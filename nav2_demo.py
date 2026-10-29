'''

source /opt/ros/jazzy/setup.bash

ros2 launch nav2_bringup cloned_multi_tb3_simulation_launch.py \
  use_rviz:=False autostart:=true \
  'robots:=robot1={x: -2.0, y: -0.5, yaw: 0.0}; robot2={x: 1.0, y: -0.5, yaw: 3.14}; robot3={x: -0.5, y: 1.5, yaw: -1.57}'


'''



'''
source /opt/ros/jazzy/setup.bash
gz sim -g

'''




'''
source /opt/ros/jazzy/setup.bash
python3 nav2_demo.py
'''

import rclpy
import math
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
from gz.transport13 import Node as GazeboNode
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.msgs10.boolean_pb2 import Boolean

## Setup
robots = {'robot1': (-2.0, -0.5, 0.0), 'robot2': (1.0, -0.5, 3.14), 'robot3': (-0.5, 1.5, -1.57)}
goal = (-0.55, -1.65)
waypoints = [(1.65, -0.55), (1.65, 0.55)]
through = [(-0.55, 2.0), (0.85, 1.8)]

def makepose(nav, x, y, yaw = 0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose

def make_navigator(name, start):
    nav = BasicNavigator(node_name=f'{name}_navigator', namespace=name)
    nav.set_parameters([Parameter('use_sim_time', value=True)])
    nav.setInitialPose(makepose(nav, start[0], start[1], start[2]))
    return nav

def show_route(name, path, waypoints, red, green, blue):
    markers = Marker_V()
    line = markers.marker.add(ns=name, id=1, type=Marker.LINE_STRIP)
    line.scale.x = line.scale.y = line.scale.z = 1.0
    for pose in path.poses:
        line.point.add(x=pose.pose.position.x, y=pose.pose.position.y, z=0.04)
    for i, pose in enumerate(waypoints):
        dot = markers.marker.add(ns=name, id=10+i, type=Marker.SPHERE)
        dot.pose.position.x = pose.pose.position.x
        dot.pose.position.y = pose.pose.position.y
        dot.pose.position.z = 0.05
        dot.scale.x = dot.scale.y = dot.scale.z = 0.20
    for marker in markers.marker:
        marker.pose.orientation.w = 1.0
        marker.material.diffuse.r = red
        marker.material.diffuse.g = green
        marker.material.diffuse.b = blue
        marker.material.diffuse.a = 1.0
        marker.material.ambient.CopyFrom(marker.material.diffuse)
    GazeboNode().request('/marker_array', markers, Marker_V, Boolean, 1000)

def main():
    rclpy.init()
    robot1 = make_navigator('robot1', robots['robot1'])
    robot2 = make_navigator('robot2', robots['robot2'])
    robot3 = make_navigator('robot3', robots['robot3'])
    bots = [robot1, robot2, robot3]
    for bot in bots:
        bot.waitUntilNav2Active()

    goal_pose = makepose(robot1, goal[0], goal[1], -1.57)
    waypoint_poses = [makepose(robot2, waypoints[0][0], waypoints[0][1], 1.57), makepose(robot2, waypoints[1][0], waypoints[1][1], 1.57)]
    through_poses = [makepose(robot3, through[0][0], through[0][1]), makepose(robot3, through[1][0], through[1][1])]

    path1 = robot1.getPath(makepose(robot1, robots['robot1'][0], robots['robot1'][1]), goal_pose)
    path2 = robot2.getPathThroughPoses(makepose(robot2, robots['robot2'][0], robots['robot2'][1]), waypoint_poses)
    path3 = robot3.getPathThroughPoses(makepose(robot3, robots['robot3'][0], robots['robot3'][1]), through_poses)
    show_route('robot1', path1, [goal_pose], 1.0, 0.0, 0.0)
    show_route('robot2', path2, waypoint_poses, 0.0, 0.3, 1.0)
    show_route('robot3', path3, through_poses, 0.0, 0.8, 0.2)

    robot1.goToPose(goal_pose)
    robot2.followWaypoints(waypoint_poses)
    robot3.goThroughPoses(through_poses)

    while not all([bot.isTaskComplete() for bot in bots]):
        for bot in bots:
            feedback = bot.getFeedback()
            if feedback and hasattr(feedback, 'distance_remaining'):
                print(f'Distance remaining: {feedback.distance_remaining:.2f} m')
        time.sleep(0.1)

    print(f'robot1 : {robot1.getResult().name}')
    print(f'robot2 : {robot2.getResult().name}')
    print(f'robot3 : {robot3.getResult().name}')
    for bot in bots:
        bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()