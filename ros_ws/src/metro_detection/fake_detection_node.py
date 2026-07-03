#!/usr/bin/env python3
# fake_detection_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class FakeDetectionPublisher(Node):
    def __init__(self):
        super().__init__('metro_detection_node')
        
        # 创建发布者，发布话题 /detections
        self.publisher_ = self.create_publisher(String, '/detections', 10)
        
        # 1秒发1次假数据
        self.timer_period = 1.0  # 秒
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('发送消息中...')

    def timer_callback(self):
        # 消息内容
        fake_data = {
            "type": "crack", # 裂缝
            "confidence": 0.95,
            "bbox": [620, 310, 760, 390] 
        }
        msg = String()
        msg.data = json.dumps(fake_data)
        
        # 推送话题
        self.publisher_.publish(msg)
        self.get_logger().info(f'这是数据 -> {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = FakeDetectionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('关停！')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()