#!/usr/bin/env python3

import airsim
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import threading
import queue
import time


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        # 连接到 AirSim 仿真器
        self.client = airsim.MultirotorClient("127.0.0.1")
        self.client.confirmConnection()

        self.bridge = CvBridge()

        # 创建图像发布器
        self.left_image_pub = self.create_publisher(
            Image, "/camera/left/image_raw", 10
        )
        self.right_image_pub = self.create_publisher(
            Image, "/camera/right/image_raw", 10
        )
        self.depth_image_pub = self.create_publisher(
            Image, "/camera/depth/image_raw", 10
        )

        # 使用队列在线程间传递图像数据
        self.image_queue = queue.Queue(maxsize=2)

        # 在单独线程中获取图像（避免阻塞 ROS 循环）
        self.fetch_thread = threading.Thread(target=self.fetch_images_loop, daemon=True)
        self.fetch_thread.start()

        # 创建定时器，以 30Hz 的频率发布图像
        self.timer = self.create_timer(1.0 / 30.0, self.publish_images)

        self.get_logger().info("Camera publisher initialized at 30Hz")

    def fetch_images_loop(self):
        """在单独线程中持续获取图像"""
        while rclpy.ok():
            try:
                t0 = time.time()

                # 批量请求所有图像
                requests = [
                    airsim.ImageRequest(
                        "CameraImage", airsim.ImageType.Scene, False, False
                    ),
                    airsim.ImageRequest(
                        "CameraRight", airsim.ImageType.Scene, False, False
                    ),
                    airsim.ImageRequest(
                        "CameraDepth", airsim.ImageType.DepthPlanar, True, False
                    ),
                ]

                responses = self.client.simGetImages(requests, vehicle_name="PX4")

                fetch_cost = time.time() - t0
                self.get_logger().info(
                    f"fetch cost: {fetch_cost:.3f}s",
                    throttle_duration_sec=1.0,
                )

                if len(responses) == 3:
                    # 如果队列满了，丢弃旧数据
                    if self.image_queue.full():
                        try:
                            self.image_queue.get_nowait()
                        except queue.Empty:
                            pass
                    self.image_queue.put(responses)

                time.sleep(0.005)

            except Exception as e:
                self.get_logger().error(f"Failed to fetch images: {e}")
                time.sleep(0.1)

    def publish_images(self):
        """从队列中取出图像并发布"""
        try:
            responses = self.image_queue.get_nowait()

            timestamp = self.get_clock().now().to_msg()

            if responses[0].width > 0:
                left_image = self.process_scene_image(responses[0])
                left_msg = self.bridge.cv2_to_imgmsg(left_image, encoding="bgr8")
                left_msg.header.stamp = timestamp
                left_msg.header.frame_id = "camera_left"
                self.left_image_pub.publish(left_msg)

            if responses[1].width > 0:
                right_image = self.process_scene_image(responses[1])
                right_msg = self.bridge.cv2_to_imgmsg(right_image, encoding="bgr8")
                right_msg.header.stamp = timestamp
                right_msg.header.frame_id = "camera_right"
                self.right_image_pub.publish(right_msg)

            if responses[2].width > 0:
                depth_image = self.process_depth_image(responses[2])
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
                depth_msg.header.stamp = timestamp
                depth_msg.header.frame_id = "camera_depth"
                self.depth_image_pub.publish(depth_msg)

        except queue.Empty:
            pass
        except Exception as e:
            self.get_logger().error(f"Failed to publish images: {e}")

    def process_scene_image(self, response):
        """处理 RGB 场景图像"""
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)
        img_bgr = img_rgb[:, :, ::-1]
        return img_bgr

    def process_depth_image(self, response):
        """处理深度图像"""
        img1d = np.array(response.image_data_float, dtype=np.float32)
        img_depth = img1d.reshape(response.height, response.width)
        return img_depth


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
