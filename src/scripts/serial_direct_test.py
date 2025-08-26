import serial
import struct
import argparse

def send_command(ser, linear_x_val, angular_z_val):
    """
    打包并发送指令到串口。
    
    :param ser: serial.Serial 对象
    :param linear_x_val: 线速度对应的 int16 值
    :param angular_z_val: 角速度对应的 int16 值
    """
    # 帧头和帧尾
    header = b'\xAA\xFE'
    footer = b'\xEA'
    
    # 使用 struct 将两个 int16 值打包成字节流（小端序）
    data_payload = struct.pack('<hh', int(linear_x_val), int(angular_z_val))
    
    # 组合成完整数据帧
    frame = header + data_payload + footer
    
    # 发送
    ser.write(frame)
    print(f"  >> 已发送: {frame.hex().upper()} | linear: {linear_x_val}, angular: {angular_z_val}")

def main():
    # 设置命令行参数解析，方便指定串口
    parser = argparse.ArgumentParser(description='交互式地向串口发送指令以测试机器人')
    parser.add_argument('--port', type=str, required=True, help='要连接的串口号, 例如 /dev/ttyUSB1')
    args = parser.parse_args()

    serial_port = args.port
    baud_rate = 115200

    try:
        # 尝试打开串口
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"成功打开串口 {serial_port}")
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {serial_port}. {e}")
        return

    # 定义测试速度值
    TEST_SPEED = 10 
    
    print("\n--- 交互式测试开始 ---")
    print("将逐一发送指令，请在每一步后按【回车】键继续。")
    input("准备好后，请按【回车】开始第一次测试...")

    try:
        # --- 1. 前进测试 ---
        print("\n【1. 测试前进】")
        send_command(ser, TEST_SPEED, 0)
        input("机器人应正在前进。观察后，请按【回车】停止...")
        send_command(ser, 0, 0) # 发送停止指令
        input("机器人应已停止。请按【回车】进行后退测试...")

        # --- 2. 后退测试 ---
        print("\n【2. 测试后退】")
        send_command(ser, -TEST_SPEED, 0)
        input("机器人应正在后退。观察后，请按【回车】停止...")
        send_command(ser, 0, 0) # 发送停止指令
        input("机器人应已停止。请按【回车】进行左转测试...")
        
        # --- 3. 左转测试 ---
        print("\n【3. 测试左转】")
        send_command(ser, 0, TEST_SPEED)
        input("机器人应正在左转。观察后，请按【回车】停止...")
        send_command(ser, 0, 0) # 发送停止指令
        input("机器人应已停止。请按【回车】进行右转测试...")

        # --- 4. 右转测试 ---
        print("\n【4. 测试右转】")
        send_command(ser, 0, -TEST_SPEED)
        input("机器人应正在右转。观察后，请按【回车】停止...")
        send_command(ser, 0, 0) # 发送停止指令
        
        print("\n\n--- 所有测试已完成 ---")

    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        # 关闭串口
        ser.close()
        print("串口已关闭。")

if __name__ == '__main__':
    main()
