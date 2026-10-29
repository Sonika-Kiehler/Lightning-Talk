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
goal = (0.0, -0.5)

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
    nav.robot_name = name
    return nav


# Extracts all the waypoints from the path and finds the total length using hypotenuse
def path_length(robot):
    length = 0
    robot_path = robot.getPath(makepose(robot, robots[robot.robot_name][0], robots[robot.robot_name][1], robots[robot.robot_name][2]), makepose(robot, goal[0], goal[1]), use_start=True)
    for i in range(1, len(robot_path.poses)):
        length += math.hypot(robot_path.poses[i].pose.position.x - robot_path.poses[i-1].pose.position.x, robot_path.poses[i].pose.position.y - robot_path.poses[i-1].pose.position.y)
    return length
# Gets the path length of every robot and returns the robot with the shortest path
def shortest_length(bots):
    len = 0
    r_name = 0
    for key in bots:
        if len == 0:
            len = path_length(key)
            r_name = key
        elif path_length(key) < len:
            len = path_length(key)
            r_name = key
    return r_name


def main():
    rclpy.init()
    robot1 = make_navigator('robot1', robots['robot1'])
    robot2 = make_navigator('robot2', robots['robot2'])
    robot3 = make_navigator('robot3', robots['robot3'])
    bots = [robot1, robot2, robot3]
    for bot in bots:
         bot.waitUntilNav2Active()
    # Sends the robot with the shortest estimated path to the goal
    r_name = shortest_length(bots)
    r_name.goToPose(makepose(r_name, goal[0], goal[1]))
    # Monitor: runs while the robot moves
    while not r_name.isTaskComplete():
        feedback = r_name.getFeedback()
        if feedback and hasattr(feedback, 'distance_remaining'):
                        print(f'Distance remaining: {feedback.distance_remaining:.2f} m')
    print(f'{r_name.robot_name} : {r_name.getResult().name}')
    for bot in bots:
        bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()