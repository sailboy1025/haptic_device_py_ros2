import asyncio
import websockets
import orjson
import numpy as np
import os
import requests

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Bool
from ament_index_python.packages import get_package_share_directory

class Inverse3Node(Node):
    def __init__(self):
        super().__init__('inverse3_node')
        self.position_publisher = self.create_publisher(PoseStamped, 'pose', 10)
        self.velocity_publisher = self.create_publisher(Vector3, 'velocity', 10)
        self.button_publisher = self.create_publisher(Joy, 'buttons', 10)
        self.use_for_sim_publisher = self.create_subscription(
            Bool, 'use_for_coppeliasim', self.sim_cb, 10)
        # Subscriptions
        self.create_subscription(Bool, '/HD_force_lock', self.force_lock_callback, 10)
        self.create_subscription(Bool, '/HD_gc_toggle', self.gc_toggle_callback, 10)
        
        self.gravity_compensation_enabled = True
        self.last_quaternion = None

        # Force lock PD parameters
        self.force_lock = True
        self.lock_center = np.array([0.03, -0.16, 0.2], dtype=np.float32)
        self.max_Kp = 80.0
        self.min_Kp = 5.0
        self.Kd = 0.002
        self.max_distance = 0.03

        self.inverse3_device_id = None

        # Load calibration matrix
        self.device_to_world = self.load_calibration()

        # Alter the frame for CoppeliaSim
        self.use_for_coppeliasim = False
    def sim_cb(self, msg: Bool):
        if msg.data is not None:
            self.use_for_coppeliasim = msg.data
    def load_calibration(self):
        try:
            package_share = get_package_share_directory('hd_py')
            cali_file = os.path.join(package_share, 'device_cal', 'inverse3_cali_param.json')
            print(cali_file)
            with open(cali_file, 'rb') as f:
                data = orjson.loads(f.read())
            matrix = np.zeros((3,3), dtype=np.float64)
            for i in range(3):
                for j in range(3):
                    matrix[i,j] = data[i][j]
            self.get_logger().info('Loaded calibration matrix.')
            return matrix
        except Exception as e:
            self.get_logger().error(f'Calibration file load failed: {e}')
            return np.identity(3)

    def force_lock_callback(self, msg: Bool):
        self.force_lock = msg.data
        self.get_logger().info(f'Force Lock: {self.force_lock}')

    def gc_toggle_callback(self, msg: Bool):
        if self.inverse3_device_id is not None:
            self.enable_gravity_compensation(self.inverse3_device_id, msg.data)

    def enable_gravity_compensation(self, device_id, enable=True, gravity_scaling_factor=1.0):
        url = "http://localhost:10000/gravity_compensation"
        payload = {
            "device_id": device_id,
            "enable": enable,
            "gravity_scaling_factor": gravity_scaling_factor
        }
        try:
            response = requests.post(url, json=payload)
            if response.status_code == 200:
                self.gravity_compensation_enabled = enable
                self.get_logger().info(f"Gravity compensation set to {enable}")
            else:
                self.get_logger().error(f"Gravity compensation request failed: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Gravity compensation request error: {e}")

    def publish_all(self, position, velocity, orientation, buttons):
        pos_msg = PoseStamped()
        pos_msg.header = Header()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = "inv3_frame"

        raw_pos = np.array([
            position.get("x", 0.0),
            position.get("y", 0.0),
            position.get("z", 0.0)
        ], dtype=np.float64)

        if self.use_for_coppeliasim:
            # Alter the frame for CoppeliaSim
            raw_pos[:2] *= -1  # Invert x and y axes
        cal_pos = self.device_to_world.T @ raw_pos 

        pos_msg.pose.position.x, pos_msg.pose.position.y, pos_msg.pose.position.z = cal_pos

        quat = [
            float(orientation.get("x", 0.0)),
            float(orientation.get("y", 0.0)),
            float(orientation.get("z", 0.0)),
            float(orientation.get("w", 1.0))
        ]
        alpha = 0.1
        if self.last_quaternion is None:
            self.last_quaternion = quat
        else:
            quat = [alpha * q + (1 - alpha) * lq for q, lq in zip(quat, self.last_quaternion)]
            self.last_quaternion = quat

        pos_msg.pose.orientation.x, pos_msg.pose.orientation.y, pos_msg.pose.orientation.z, pos_msg.pose.orientation.w = quat

        self.position_publisher.publish(pos_msg)

        vel_msg = Vector3()
        vel_msg.x, vel_msg.y, vel_msg.z = (
            float(velocity.get(axis, 0.0)) for axis in ("x", "y", "z")
        )
        self.velocity_publisher.publish(vel_msg)

        button_msg = Joy()
        button_msg.header = Header()
        button_msg.header.stamp = self.get_clock().now().to_msg()
        button_msg.buttons = [int(b) for b in buttons.values()] if buttons else []
        self.button_publisher.publish(button_msg)

    def compute_force(self, current_pos, current_vel):
        force = np.zeros(3, dtype=np.float32)
        distance = np.linalg.norm(self.lock_center - current_pos)
        for i in range(3):
            Kp = self.max_Kp if distance <= self.max_distance else self.min_Kp
            error = self.lock_center[i] - current_pos[i]
            force[i] = Kp * error - self.Kd * current_vel[i]
        return force

async def websocket_loop(node: Inverse3Node):
    uri = 'ws://localhost:10001'
    first_message = True

    async with websockets.connect(uri) as ws:
        while rclpy.ok():
            response = await ws.recv()
            data = orjson.loads(response)

            inverse3_devices = data.get("inverse3", [])
            verse_grip_devices = data.get("wireless_verse_grip", [])
            inverse3_data = inverse3_devices[0] if inverse3_devices else {}
            verse_grip_data = verse_grip_devices[0] if verse_grip_devices else {}

            if first_message:
                first_message = False
                if not inverse3_data:
                    node.get_logger().error("No Inverse3 device found. Retrying...")
                    await asyncio.sleep(1)
                    continue
                if not verse_grip_data:
                    node.get_logger().error("No Wireless Verse Grip device found. Retrying...")
                    await asyncio.sleep(1)
                    continue
                node.inverse3_device_id = inverse3_data.get('device_id')
                node.get_logger().info(f"Inverse3 device ID: {node.inverse3_device_id}")
                node.get_logger().info(f"Wireless Verse Grip device ID: {verse_grip_data.get('device_id')}")
                node.get_logger().info(f"Wireless Verse Grip Battery Level: {verse_grip_data.get('state', {}).get('battery_level', {})}")
                node.enable_gravity_compensation(node.inverse3_device_id, enable=True)

            position = inverse3_data["state"].get("cursor_position", {})
            velocity = inverse3_data["state"].get("cursor_velocity", {})
            buttons = verse_grip_data.get("state", {}).get("buttons", {})
            orientation = verse_grip_data.get("state", {}).get("orientation", {})

            node.publish_all(position, velocity, orientation, buttons)

            current_pos = np.array([
                float(position.get("x", 0.0)),
                float(position.get("y", 0.0)),
                float(position.get("z", 0.0))
            ], dtype=np.float32)

            current_vel = np.array([
                float(velocity.get("x", 0.0)),
                float(velocity.get("y", 0.0)),
                float(velocity.get("z", 0.0))
            ], dtype=np.float32)

            if node.force_lock:
                force = node.compute_force(current_pos, current_vel)
            else:
                force = np.zeros(3, dtype=np.float32)

            request_msg = {
                "inverse3": [
                    {
                        "device_id": node.inverse3_device_id,
                        "commands": {
                            "set_cursor_force": {
                                "values": {
                                    "x": float(force[0]),
                                    "y": float(force[1]),
                                    "z": float(force[2])
                                }
                            }
                        }
                    }
                ]
            }
            await ws.send(orjson.dumps(request_msg))

def main():
    rclpy.init()
    node = Inverse3Node()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, executor.spin)
    try:
        loop.run_until_complete(websocket_loop(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
