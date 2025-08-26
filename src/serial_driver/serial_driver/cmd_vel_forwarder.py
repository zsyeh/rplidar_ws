#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct # 用于将数字打包成字节

class CmdVelToSerial(Node):
    """
    该节点订阅/cmd_vel话题,并将linear.x和angular.z的值处理后通过串口发送。
    """
    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')

        # 声明并获取参数
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        
        self.serial_port_ = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate_ = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.get_logger().info(f'正在尝试连接串口: {self.serial_port_} @ {self.baud_rate_}bps')

        # 初始化串口连接
        try:
            self.ser_ = serial.Serial(
                self.serial_port_,
                self.baud_rate_,
                timeout=1.0
            )
            self.get_logger().info('串口连接成功.')
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口 {self.serial_port_}: {e}')
            # 退出节点
            self.destroy_node()
            rclpy.shutdown()
            return

        # 创建/cmd_vel话题的订阅者
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10  # QoS profile depth
        )
        self.get_logger().info('节点已启动，等待/cmd_vel消息...')

    def cmd_vel_callback(self, msg: Twist):
        """
        接收到/cmd_vel消息时的回调函数
        """
        # 1. 提取 linear.x 和 angular.z
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 2. 数据处理：乘10并转为int16
        # 注意: int() 会直接截断小数部分
        # int16的范围是 -32768 到 32767。如果处理后的值超出此范围，struct.pack会报错。
        # 在实际应用中，您可能需要添加范围检查。
        data_x = int(linear_x * 10.0)
        data_z = int(angular_z * 10.0)

        # 3. 构建数据帧: AA FE [data_x(2B)] [data_z(2B)] EA
        header = b'\xAA\xFE'
        footer = b'\xEA'
        
        # 使用struct.pack将两个int16打包成字节流
        # '<' 表示小端字节序 (little-endian)
        # 'h' 表示有符号16位整型 (signed short)
        try:
            data_payload = struct.pack('<hh', data_x, data_z)
            
            # 组合成完整的数据帧
            frame = header + data_payload + footer
            
            # 4. 通过串口发送
            if self.ser_.is_open:
                self.ser_.write(frame)
                # 打印日志用于调试，以十六进制格式显示发送的数据
                self.get_logger().info(f'发送数据: {frame.hex().upper()} | linear.x: {linear_x:.2f} -> {data_x}, angular.z: {angular_z:.2f} -> {data_z}')
            else:
                self.get_logger().warn('串口未打开，无法发送数据。')

        except struct.error as e:
            self.get_logger().error(f'打包数据时出错: {e}. 请检查输入速度是否在int16范围内。')
            self.get_logger().error(f'尝试打包的值: data_x={data_x}, data_z={data_z}')
            
    def destroy_node(self):
        """
        节点关闭时关闭串口连接
        """
        if hasattr(self, 'ser_') and self.ser_.is_open:
            self.ser_.close()
            self.get_logger().info('串口已关闭。')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断。')
    finally:
        # 销毁节点，这会调用destroy_node方法
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
