#!/usr/bin/env python3
import rclpy
import time
import json
import os
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot3_msgs.action import FoodDelivery
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool
from std_srvs.srv import Trigger


import asyncio
from functools import partial

class FoodDeliveryActionServer(Node):

    def __init__(self):
        super().__init__('food_delivery_server')

        # Create and keep the navigator as an instance variable
        self.navigator = BasicNavigator()

        # Set initial pose using BasicNavigator
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0  
        self.navigator.setInitialPose(initial_pose)

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

        # Action server
        self._action_server = ActionServer(
            self,
            FoodDelivery,
            'food_delivery',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )


        self.confirm = self.create_client(Trigger,'confirm_order')


        # Waypoints (loaded from JSON)
        self.waypoints = self.load_waypoints()


    def goal_navigator(self, navigator, goal_pose):
        """
        Uses the BasicNavigator to drive toward the goal_pose.
        goal_pose is expected to be a list: [x, y, yaw_deg]
        """
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = goal_pose[0]
        goal_pose_msg.pose.position.y = goal_pose[1]

        # Convert yaw (goal_pose[2]) to quaternion
        q = quaternion_from_euler(0, 0, goal_pose[2])
        goal_pose_msg.pose.orientation.x = q[0]
        goal_pose_msg.pose.orientation.y = q[1]
        goal_pose_msg.pose.orientation.z = q[2]
        goal_pose_msg.pose.orientation.w = q[3]

        self.get_logger().info(
            f"Navigating to x={goal_pose[0]:.2f}, y={goal_pose[1]:.2f}, yaw={goal_pose[2]:.2f}"
        )

        # Command the robot to go to goal_pose_msg
        navigator.goToPose(goal_pose_msg)

        # Wait until task completes with a small sleep to avoid high CPU usage
        while not navigator.isTaskComplete():
            time.sleep(0.1)

        what_happened = navigator.getResult()

        if what_happened == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation succeeded.')
        elif what_happened == TaskResult.CANCELED:
            self.get_logger().warn('Navigation canceled.')
        elif what_happened == TaskResult.FAILED:
            self.get_logger().error('Navigation failed.')
        else:
            self.get_logger().warn('Unexpected navigation result.')

        return what_happened == TaskResult.SUCCEEDED

    def load_waypoints(self):
        config_file = os.path.join(
            get_package_share_directory('turtlebot3_example'),
            'config',
            'waypoints.json'
        )
        with open(config_file, 'r') as f:
            data = json.load(f)

        waypoints = {}
        for key, coords in data.items():
            waypoints[key] = coords  # Each coords is a list like [x, y, yaw_deg]

        self.get_logger().info(f"Waypoints loaded: {list(waypoints.keys())}")
        return waypoints

    def wait_for_navigation(self, goal_name):
        
        if not self.confirm.wait_for_service(timeout_sec=15.0):
            self.get_logger().warn(f'{goal_name} confirmation service not available.')
            return False

        req = Trigger.Request()
        future = self.confirm.call_async(req)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'{goal_name} order confirmed.')
            return True
        else:
            self.get_logger().warn(f'{goal_name} order rejected or failed.')
            return False

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received delivery goal for tables: {goal_request.table_ids}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        # Reuse the instance navigator
        navigator = self.navigator
        feedback_msg = FoodDelivery.Feedback()
        result = FoodDelivery.Result()
        count = 0

        table_ids = goal_handle.request.table_ids
        require_kitchen_confirmation = goal_handle.request.require_kitchen_confirmation
        require_table_confirmation = goal_handle.request.require_table_confirmation
        return_to_kitchen_after_table = goal_handle.request.return_to_kitchen_after_table

        # Go to kitchen first
        feedback_msg.current_state = "Going to kitchen"
        goal_handle.publish_feedback(feedback_msg)

        what_happened = self.goal_navigator(navigator, self.waypoints["kitchen"])
        future=self.wait_for_navigation("kitchen")

        if not what_happened:
            result.success = False
            result.message = "Failed to reach kitchen"
            return result

        if future == False:
            what_happened = self.goal_navigator(navigator, self.waypoints["home"])
            result.success=False
            result.message = "failed to confirm the order"
            return result

        for table_id in table_ids:
            if goal_handle.is_cancel_requested:
                what_happened = self.goal_navigator(navigator, self.waypoints["home"])
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled"
                return result

            feedback_msg.current_state = f"Going to {table_id}"
            goal_handle.publish_feedback(feedback_msg)

            what_happened = self.goal_navigator(navigator, self.waypoints[table_id])
            future=self.wait_for_navigation("table no{table_id}")
            if not what_happened:
                continue
            if future == False:
                count=count+1
                self.get_logger().info(f"Skipping {table_id} due to no confirmation")
                continue

        if count>0:
            what_happened = self.goal_navigator(navigator, self.waypoints["kitchen"])

        what_happened = self.goal_navigator(navigator, self.waypoints["home"])

        result.success = True
        result.message = "Delivery task completed"
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FoodDeliveryActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
