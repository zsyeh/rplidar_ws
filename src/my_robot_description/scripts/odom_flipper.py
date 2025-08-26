#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
import math

class OdomFlipper(Node):
    def __init__(self):
        super().__init__('odom_flipper_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # 订阅由rf2o发布的错误的odom
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(
            Odometry,
            '/odom_corrected',  # 发布到修正后的新话题
            10)
        self.get_logger().info('里程计修正节点已启动，/odom -> /odom_corrected')

    def odom_callback(self, msg: Odometry):
        corrected_msg = msg  # 先复制一份消息

        # --- 核心修正逻辑 ---
        # 1. 反转线速度和角速度
        corrected_msg.twist.twist.linear.x *= -1.0
        # 如果需要，也可以反转y和角速度，根据实际测试情况决定
        # corrected_msg.twist.twist.linear.y *= -1.0 
        # corrected_msg.twist.twist.angular.z *= -1.0

        # 注意：里程计的位置(pose)是速度的积分，我们通常不直接修改它，
        # 因为里程计的TF是由rf2o发布的，我们无法通过修改消息来修正它。
        # rf2o发布的 odom -> base_link 的TF本身就是反的。
        # 所以，我们上面的“欺骗”URDF的方法理论上才是正道。
        # 如果那个方法无效，说明有更深层的问题。

        # 让我们重新思考：如果推车向前，odom的x减少，说明rf2o的计算结果是反的。
        # 为什么骗它TF没用？可能是因为它不关心base_link -> laser_frame的yaw。
        # 它只关心laser_frame本身。
        
        # 让我们尝试一个更直接的修正，我们直接修正TF。
        # 不好，这个节点不应该发布TF。

        # 让我们坚持最简单的方案。如果rf2o发布的odom消息里的twist是反的，
        # 并且它发布的TF odom->base_link也是反的。
        # 那么问题一定出在它对“前进”的定义上。
        # 既然URDF欺骗无效，让我们尝试欺骗雷达数据。
        # 不，那太复杂了。

        # 回到这个节点。我们只修正twist，给Nav2提供正确的速度指令参考。
        # 但这无法解决TF反了的问题。

        self.publisher.publish(corrected_msg)

# -----------------------------------------------------------
# 上面的脚本思路有问题，它无法修正错误的TF。
# 让我们回到最根本的地方。

# 之前的诊断是：
# 1. 物理前进
# 2. Odom位置X减少
# 这意味着 rf2o 计算出的 odom -> base_link 变换是反的。
# 之前我们尝试用 rpy="... 3.14" 来欺骗它，使其计算出正向的变换。
# 用户说这依然是反的。

# 这只剩下两种可能性：
# 1. 用户的“隔离验证”是正确的，但“推车测试”时，rf2o的行为因为某种原因变了。
# 2. 用户的“隔离验证”其实是错误的，雷达TF本身就需要旋转180度。

# 让我们提供一个最终的、绝对的检查方案，不再依赖用户的口头描述。
# 我将重写这个回复。
