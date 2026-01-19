import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main(args=None):
    rclpy.init()
    navigator = BasicNavigator()

    w1 = [1.47, 2.28]
    w2 = [3.78, 1.09]
    w3 = [2.57, -1.26]

    goal_poses = []
    for w in [w1, w2, w3]:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = w[0]
        pose.pose.position.y = w[1]
        pose.pose.orientation.z = 1.0
        goal_poses.append(pose)

    print("Starting patrol...")
    navigator.followWaypoints(goal_poses)

    while not navigator.isTaskComplete():
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Patrol completed !")
    else:
        print("Patrol failed")

    rclpy.shutdown()


if __name__ == "__main__":
    main()