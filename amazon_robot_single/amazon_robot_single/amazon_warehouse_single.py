import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


from geometry_msgs.msg import PoseStamped
from amazon_robot_msg.action import FollowTargets

pallets = {
    'pallet_1': {'x_pose': 3.64, 'y_pose': 0.63, 'z_pose': 0.01},
    'pallet_2': {'x_pose': 3.59, 'y_pose': -1.11, 'z_pose': 0.01},
    'pallet_3': {'x_pose': 3.51, 'y_pose': -2.84, 'z_pose': 0.01},
    'pallet_4': {'x_pose': 3.49, 'y_pose': -4.68, 'z_pose': 0.01},
    'pallet_5': {'x_pose': 3.64, 'y_pose': -6.91, 'z_pose': 0.01},
    'pallet_6': {'x_pose': 3.64, 'y_pose': -8.88, 'z_pose': 0.01},
}

storage_locations = {
    'storage_location_1': {'x_pose': -5.84, 'y_pose': -3.35, 'z_pose': 0.01},
    'storage_location_2': {'x_pose': -5.84, 'y_pose': 1.12, 'z_pose': 0.01},
    'storage_location_3': {'x_pose': -5.84, 'y_pose': -7.76, 'z_pose': 0.01},
}

free_area = {
    'right_end_of_corridor': {'x_pose': 1.35, 'y_pose': -6.78, 'z_pose': 0.01},
    'left_end_of_corridor': {'x_pose': 0.92, 'y_pose': 6.45, 'z_pose': 0.01},
}

lift_stages = {'load': 2, 'unload': -2, 'half_load': 1, 'half_unload': -1, 'unchanged': 0}


# Helper function to convert x,y,z pose to PoseStamped pose format.
def get_pose_stamped(x_pose, y_pose, z_pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose.position.x = x_pose
    pose_stamped.pose.position.y = y_pose
    pose_stamped.pose.position.z = z_pose
    return pose_stamped


class FollowTargetActionClient(Node):
    def __init__(self):
        super().__init__('follow_target_action_client')
        self.action_client = ActionClient(self, FollowTargets, '/FollowTargets')
    
    def send_goal(self, poses, load):
        if len(poses) != len(load):
            self.get_logger().warn('Length of Pose not same as Loads')
            return
        
        self.current_goal_length = len(poses)
        goal = FollowTargets.Goal()
        goal.poses = poses
        goal.load = load
        self.get_logger().info("waiting for Action Server")
        self.action_client.wait_for_server()
        self.get_logger().info("Found Action Server")
        self.future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Currently Executing: {0}'.format(feedback.current_waypoint))

    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Mission Completed')
        self.get_logger().info('No of Missed Waypoints: {0}'.format(len(result.missed_waypoints)))
        rclpy.shutdown()

class AmazonRobot(FollowTargetActionClient):
    def __init__(self):
        super().__init__()
    
    def move_pallets(self):
        poses_robot = [get_pose_stamped(**pallets['pallet_1']),
                       get_pose_stamped(**storage_locations['storage_location_1']),
                       
                       get_pose_stamped(**free_area['right_end_of_corridor']),
                       ]
        loads_robot = [
            lift_stages['load'],
            lift_stages['unload'],

            lift_stages['unload'],
        ]

        self.get_logger().info("Sending target goals to the robot")
        self.send_goal(poses_robot, loads_robot)



def main(args=None):
    rclpy.init(args=args)

    robot = AmazonRobot()
    robot.move_pallets()

    rclpy.spin(robot)

if __name__ == '__main__':
    main()
