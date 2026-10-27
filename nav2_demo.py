# Create 3 turtle bots
# Have each bot interact with Nav2 core functions we menionted 
# - BasicNavigator
# - goToPose
# - goThroughPoses
# - followWaypoints
# - getPath
# - spin

#Monitor tasks
# - isTaskComplete
# - getFeedback
# - getResult


# Robot actions
# - mutlti robot - avoid paths
# robot able to consume each other's data 
# come up with algo to handle collisons

## RUN THIS IN TERMINALS:
### Terminal 1:
#### conda deactivate
#### pkill -f gz
#### pkill -f rviz2
#### ros2 launch nav2_bringup cloned_multi_tb3_simulation_launch.py \use_rviz:=False autostart:=true \robots:="robot1={x: -2.0, y: -0.5, yaw: 0.0}; robot2={x: 1.0, y: -0.5, yaw: 3.14}; robot3={x: -0.5, y: 1.5, yaw: -1.57}"
### Terminal 2:
#### conda deactivate
#### gz sim -g
### Terminal 3:
#### conda deactivate
#### python3 nav2_demo.py

import rclpy
import math
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

## Setup
robots = {'robot1': (-2.0, -0.5, 0.0), 'robot2': (1.0, -0.5, 3.14), 'robot3': (-0.5, 1.5, -1.57)}
goal = (1.0, -0.5)
waypoints = [(0.0, -1.0), (-2.0, -0.5)]
through = [(-0.5, 0.5), (-0.5, -1.0)]

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
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    nav.setInitialPose(makepose(nav, start[0], start[1], start[2]))
    return nav


def main():
    rclpy.init()
    robot1 = make_navigator('robot1', robots['robot1'])
    robot2 = make_navigator('robot2', robots['robot2'])
    robot3 = make_navigator('robot3', robots['robot3'])
    bots = [robot1, robot2, robot3]
    for bot in bots:
        bot.waitUntilNav2Active()

    robot1.goToPose(makepose(robot1, goal[0], goal[1]))
    robot2. followWaypoints([makepose(robot2, waypoints[0][0], waypoints[0][1]), makepose(robot2, waypoints[1][0], waypoints[1][1])])
    robot3.goThroughPoses([makepose(robot3, through[0][0], through[0][1]), makepose(robot3, through[1][0], through[1][1])])

    # Monitor: runs while the robot moves
    while not all([bot.isTaskComplete() for bot in bots]):
        for bot in bots:
            feedback = bot.getFeedback()
            if feedback and hasattr(feedback, 'distance_remaining'):
                print(f'Distance remaining: {feedback.distance_remaining:.2f} m')

    for bot, name in zip(bots, robots):
        print(f'{name} : {bot.getResult().name}')
    for bot in bots:
        bot.spin(spin_dist = 6.28)
    while not all([bot.isTaskComplete() for bot in bots]):
        pass

    for bot in bots:
        bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()