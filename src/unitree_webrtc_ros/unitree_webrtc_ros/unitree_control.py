#!/usr/bin/env python3
"""
ROS 2 node for controlling Unitree Go2 robot via WebRTC.
Subscribes to cmd_vel and provides sport mode command services.
"""

import asyncio
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD


class UnitreeControlNode(Node):
    """ROS 2 node for Unitree Go2 robot control via WebRTC."""

    def __init__(self):
        super().__init__('unitree_control')

        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.8.181')
        self.declare_parameter('connection_method', 'LocalSTA')
        self.declare_parameter('control_mode', 'sport_cmd')  # Options: 'sport_cmd' or 'wireless_controller'

        # Get parameters
        self.robot_ip = self.get_parameter('robot_ip').value
        connection_method_str = self.get_parameter('connection_method').value
        self.control_mode = self.get_parameter('control_mode').value

        # Map connection method string to enum
        connection_method_map = {
            'LocalAP': WebRTCConnectionMethod.LocalAP,
            'LocalSTA': WebRTCConnectionMethod.LocalSTA,
            'Remote': WebRTCConnectionMethod.Remote
        }
        self.connection_method = connection_method_map.get(
            connection_method_str, WebRTCConnectionMethod.LocalSTA
        )

        self.get_logger().info(f'Connecting to robot at {self.robot_ip} using {connection_method_str}')
        self.get_logger().info(f'Control mode: {self.control_mode}')

        # Initialize WebRTC connection
        self.conn = None
        self.loop = None
        self.connected = threading.Event()

        # Start connection in background thread
        self.connection_thread = threading.Thread(target=self._connection_worker, daemon=True)
        self.connection_thread.start()

        # Wait for connection
        if not self.connected.wait(timeout=30.0):
            self.get_logger().error('Failed to connect to robot within timeout')
            raise RuntimeError('Connection timeout')

        self.get_logger().info('Successfully connected to robot')

        # QoS profile for cmd_vel (use best effort for real-time control)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )

        # Create services for sport commands
        self.standup_srv = self.create_service(Trigger, 'standup', self.standup_callback)
        self.liedown_srv = self.create_service(Trigger, 'liedown', self.liedown_callback)
        self.hello_srv = self.create_service(Trigger, 'hello', self.hello_callback)
        self.stretch_srv = self.create_service(Trigger, 'stretch', self.stretch_callback)
        self.recovery_stand_srv = self.create_service(Trigger, 'recovery_stand', self.recovery_stand_callback)

        self.get_logger().info('Unitree control node started')
        self.get_logger().info('Subscribed to: cmd_vel')
        self.get_logger().info('Services: standup, liedown, hello, stretch, recovery_stand')

    def _connection_worker(self):
        """Background thread for asyncio event loop and WebRTC connection."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        try:
            # Create connection
            self.conn = UnitreeWebRTCConnection(
                self.connection_method,
                ip=self.robot_ip
            )

            # Connect
            self.loop.run_until_complete(self.conn.connect())

            # Signal that connection is ready
            self.connected.set()

            # Keep the loop running
            self.loop.run_forever()
        except Exception as e:
            self.get_logger().error(f'Connection error: {e}')
            self.connected.set()  # Release waiting thread even on error
        finally:
            self.loop.close()

    def cmd_vel_callback(self, msg: TwistStamped):
        """Handle incoming cmd_vel messages."""
        if not self.conn or not self.loop:
            self.get_logger().warn('Connection not ready, ignoring cmd_vel')
            return

        x, y, yaw = msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z

        # Choose control mode based on parameter
        if self.control_mode == 'wireless_controller':
            # WebRTC coordinate mapping for wireless controller:
            # lx - Positive right, negative left (maps to ROS y)
            # ly - Positive forward, negative backwards (maps to ROS x)
            # rx - Positive rotate right, negative rotate left (maps to ROS yaw)
            async def async_move():
                self.conn.datachannel.pub_sub.publish_without_callback(
                    RTC_TOPIC["WIRELESS_CONTROLLER"],
                    data={"lx": -y, "ly": x, "rx": -yaw, "ry": 0},
                )
        else:  # sport_cmd (default)
            # SPORT_CMD["Move"] coordinate mapping:
            # x - forward/backward
            # y - left/right
            # z - yaw rotation
            async def async_move():
                await self.conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["SPORT_MOD"],
                    {
                        "api_id": SPORT_CMD["Move"],
                        "parameter": {"x": x, "y": y, "z": yaw}
                    }
                )

        try:
            future = asyncio.run_coroutine_threadsafe(async_move(), self.loop)
            if self.control_mode == 'sport_cmd':
                future.result()  # Wait for sport_cmd
            # wireless_controller uses publish_without_callback, no need to wait
        except Exception as e:
            self.get_logger().error(f'Failed to send cmd_vel: {e}')

    def _execute_sport_command(self, command_name: str, api_id: int, parameter: dict = None) -> Trigger.Response:
        """Execute a sport mode command."""
        response = Trigger.Response()

        if not self.conn or not self.loop:
            response.success = False
            response.message = 'Connection not ready'
            return response

        try:
            request_data = {"api_id": api_id}
            if parameter:
                request_data["parameter"] = parameter

            async def async_command():
                await self.conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["SPORT_MOD"],
                    request_data
                )

            future = asyncio.run_coroutine_threadsafe(async_command(), self.loop)
            future.result(timeout=5.0)

            response.success = True
            response.message = f'{command_name} command sent successfully'
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Failed to execute {command_name}: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def standup_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Service callback to make robot stand up."""
        return self._execute_sport_command('StandUp', SPORT_CMD["StandUp"])

    def liedown_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Service callback to make robot lie down."""
        return self._execute_sport_command('StandDown', SPORT_CMD["StandDown"])

    def hello_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Service callback to make robot wave hello."""
        return self._execute_sport_command('Hello', SPORT_CMD["Hello"])

    def stretch_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Service callback to make robot stretch."""
        return self._execute_sport_command('Stretch', SPORT_CMD["Stretch"])

    def recovery_stand_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Service callback to recovery stand position."""
        return self._execute_sport_command('RecoveryStand', SPORT_CMD["RecoveryStand"])

    def destroy_node(self):
        """Clean up resources before node shutdown."""
        self.get_logger().info('Shutting down...')

        # Disconnect WebRTC
        if self.conn and self.loop:
            try:
                async def async_disconnect():
                    await self.conn.disconnect()

                asyncio.run_coroutine_threadsafe(async_disconnect(), self.loop)
            except Exception as e:
                self.get_logger().error(f'Error during disconnect: {e}')

        # Stop event loop
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)

        # Wait for thread to finish
        if self.connection_thread.is_alive():
            self.connection_thread.join(timeout=2.0)

        super().destroy_node()


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)

    try:
        node = UnitreeControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
