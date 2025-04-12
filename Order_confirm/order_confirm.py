#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class OrderConfirmationService(Node):
    def __init__(self):
        super().__init__('order_confirmation_service')
        # Create a service named 'confirm_order' using the Trigger service type.
        self.srv = self.create_service(Trigger, 'confirm_order', self.handle_confirm_order)
        self.get_logger().info("Order Confirmation Service is ready.")

    def handle_confirm_order(self, request, response):
        # Your custom confirmation logic here.
        # For now, we simply confirm the order.
        self.get_logger().info("Confirmation service triggered!")
        response.success = True
        response.message = "Order confirmed successfully."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = OrderConfirmationService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

