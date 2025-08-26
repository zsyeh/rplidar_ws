#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
from tf_transformations import euler_from_quaternion

class TfCheckerNode(Node):
    """
    一个用于周期性检查并打印关键TF变换的节点。
    """
    def __init__(self):
        super().__init__('tf_checker_node')
        
        # 初始化TF2的缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 创建一个定时器，每秒执行一次回调函数
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("TF监视器已启动，每秒刷新一次 'map'->'odom' 和 'odom'->'base_link' 的关系。")

    def timer_callback(self):
        """
        定时器回调函数，用于查找并打印TF变换。
        """
        self.print_transform('odom', 'base_link')
        self.print_transform('map', 'odom')
        print("-" * 40) # 打印分隔线

    def print_transform(self, parent_frame, child_frame):
        """
        查找并以可读格式打印两个frame之间的变换。
        """
        try:
            # 查找最新的可用变换
            trans = self.tf_buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time())
            
            # 提取位置信息
            pos = trans.transform.translation
            
            # 提取姿态信息（四元数）
            quat = trans.transform.rotation
            
            # 将四元数转换为欧拉角
            roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            
            # 将弧度转换为角度
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            
            # 打印信息
            self.get_logger().info(
                f"Tansform {parent_frame} -> {child_frame}:\n"
                f"  - Position (XYZ): [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] (meters)\n"
                f"  - Orientation (RPY): [{roll_deg:.2f}, {pitch_deg:.2f}, {yaw_deg:.2f}] (degrees)"
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"无法获取从 '{parent_frame}' 到 '{child_frame}' 的变换: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TfCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
