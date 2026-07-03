# e:\develop\stm32\robotArm\robot_arm.py
import serial
import time
import argparse
from typing import List, Tuple, Optional

class RobotArm:
    """机械臂控制类 - 通过串口与 STM32 通信"""
    
    def __init__(self, port: str = "COM3", baudrate: int = 115200, timeout: float = 1.0):
        """
        初始化机械臂控制类
        
        Args:
            port: 串口号，如 "COM3" 或 "/dev/ttyUSB0"
            baudrate: 波特率，默认 115200
            timeout: 串口超时时间（秒），默认 1.0
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.connected = False
        
    def connect(self) -> bool:
        """
        连接到机械臂
        
        Returns:
            bool: 连接成功返回 True，失败返回 False
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # 等待串口初始化
            
            # 清空缓冲区
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            self.connected = True
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
            
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        print("Disconnected")
    
    def _send_command(self, command: str) -> List[str]:
        """
        发送命令并接收响应
        
        Args:
            command: 要发送的命令字符串
            
        Returns:
            List[str]: 返回的响应行列表
        """
        if not self.connected or not self.ser:
            raise RuntimeError("Not connected to robot arm")
        
        # 发送命令
        cmd = command.strip() + "\n"
        self.ser.write(cmd.encode())
        
        # 等待响应
        time.sleep(0.1)
        
        # 读取响应
        responses = []
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                responses.append(line)
        
        return responses
    
    def get_joints(self) -> Optional[List[float]]:
        """
        获取当前所有关节的角度
        
        Returns:
            List[float]: 6 个关节的角度列表 [j1, j2, j3, j4, j5, j6]
                        失败返回 None
        """
        try:
            responses = self._send_command("get_joints")
            
            # 解析响应，查找 "Joints:" 开头的行
            for line in responses:
                if line.startswith("Joints:"):
                    # 提取角度值
                    parts = line.replace("Joints:", "").strip().split()
                    angles = [float(x) for x in parts]
                    if len(angles) == 6:
                        return angles
            
            print("Failed to parse joint angles from response")
            return None
            
        except Exception as e:
            print(f"Error getting joints: {e}")
            return None
    
    def set_joints(self, angles: List[float], wait: bool = True) -> bool:
        """
        设置所有关节的目标角度
        
        Args:
            angles: 6 个关节的目标角度列表 [j1, j2, j3, j4, j5, j6]
            wait: 是否等待运动完成，默认 True
            
        Returns:
            bool: 命令发送成功返回 True
        """
        if len(angles) != 6:
            raise ValueError("Angles must be a list of 6 values")
        
        try:
            # 构建命令
            cmd = f"set_joints {' '.join(f'{a:.2f}' for a in angles)}"
            responses = self._send_command(cmd)
            
            # 检查是否有错误
            for line in responses:
                if "ERROR" in line:
                    print(f"Error setting joints: {line}")
                    return False
            
            print(f"Set joints to: {angles}")
            
            if wait:
                print("Waiting for motion to complete...")
                time.sleep(2)  # 等待运动完成（可根据实际情况调整）
            
            return True
            
        except Exception as e:
            print(f"Error setting joints: {e}")
            return False
    
    def get_eepose(self) -> Optional[dict]:
        """
        获取末端位姿（位置和欧拉角姿态）
        
        Returns:
            dict: 包含位置和姿态的字典
                {
                    'x': X 位置 (mm),
                    'y': Y 位置 (mm),
                    'z': Z 位置 (mm),
                    'roll': 翻滚角 (度),
                    'pitch': 俯仰角 (度),
                    'yaw': 偏航角 (度)
                }
                失败返回 None
        """
        try:
            responses = self._send_command("get_eepose")
            
            # 解析响应，查找 Pose 行
            for line in responses:
                if line.startswith("Pose:"):
                    # 提取位姿值
                    parts = line.replace("Pose:", "[]").strip()
                    # 解析方括号内的值
                    start = parts.find('[')
                    end = parts.find(']')
                    if start != -1 and end != -1:
                        values_str = parts[start+1:end]
                        values = [float(x.strip()) for x in values_str.split(',')]
                        
                        if len(values) == 6:
                            return {
                                'x': values[0],
                                'y': values[1],
                                'z': values[2],
                                'roll': values[3],
                                'pitch': values[4],
                                'yaw': values[5]
                            }
            
            print("Failed to parse end-effector pose from response")
            return None
            
        except Exception as e:
            print(f"Error getting end-effector pose: {e}")
            return None
    
    def forward_kinematics(self, joint_angles: List[float]) -> Optional[dict]:
        """
        正运动学求解 - 根据关节角度计算末端位姿
        
        Args:
            joint_angles: 6 个关节的角度列表 [j1, j2, j3, j4, j5, j6]
            
        Returns:
            dict: 末端位姿字典
                {
                    'x': X 位置 (mm),
                    'y': Y 位置 (mm),
                    'z': Z 位置 (mm),
                    'roll': 翻滚角 (度),
                    'pitch': 俯仰角 (度),
                    'yaw': 偏航角 (度)
                }
                失败返回 None
        """
        try:
            # 构建 fk 命令
            joints_str = ' '.join(f'{j:.2f}' for j in joint_angles)
            cmd = f"fk {joints_str}"
            responses = self._send_command(cmd)
            
            # 检查是否有错误
            for line in responses:
                if "ERROR" in line:
                    print(f"Error in forward kinematics: {line}")
                    return None
            
            # 解析 FK Result 行
            for line in responses:
                if line.startswith("FK Result:"):
                    # 提取：X=200.00 Y=-50.00 Z=150.00 Roll=0.00 Pitch=45.00 Yaw=90.00
                    parts = line.replace("FK Result:", "").strip().split()
                    pose = {}
                    for part in parts:
                        if '=' in part:
                            key, value = part.split('=')
                            pose[key.lower()] = float(value)
                    
                    if len(pose) == 6:
                        print(f"Forward kinematics result: {pose}")
                        return pose
            
            print("No FK result found in response")
            return None
                
        except Exception as e:
            print(f"Error in forward kinematics: {e}")
            return None
    
    def inverse_kinematics(self, target_pose: dict, current_joints: Optional[List[float]] = None) -> Optional[List[float]]:
        """
        逆运动学求解 - 根据目标位姿计算关节角度（不执行运动）
        
        Args:
            target_pose: 目标位姿字典
                {
                    'x': X 位置 (mm),
                    'y': Y 位置 (mm),
                    'z': Z 位置 (mm),
                    'roll': 翻滚角 (度),
                    'pitch': 俯仰角 (度),
                    'yaw': 偏航角 (度)
                }
            current_joints: 当前关节角度列表（可选），用于选择最优解
                如果提供，会选择最接近当前关节值的解
                如果不提供，使用 STM32 实时读取的当前值
                
        Returns:
            List[float]: 6 个关节的角度列表，失败返回 None
        """
        try:
            # 构建命令
            x = target_pose['x']
            y = target_pose['y']
            z = target_pose['z']
            roll = target_pose['roll']
            pitch = target_pose['pitch']
            yaw = target_pose['yaw']
            
            # 如果提供了当前关节值，添加到命令中
            if current_joints:
                if len(current_joints) != 6:
                    raise ValueError("current_joints must be a list of 6 values")
                joints_str = ' '.join(f'{j:.2f}' for j in current_joints)
                cmd = f"get_ik {x:.2f} {y:.2f} {z:.2f} {roll:.2f} {pitch:.2f} {yaw:.2f} --current {joints_str}"
            else:
                cmd = f"get_ik {x:.2f} {y:.2f} {z:.2f} {roll:.2f} {pitch:.2f} {yaw:.2f}"
            
            responses = self._send_command(cmd)
            
            # 检查是否有错误
            for line in responses:
                if "ERROR" in line:
                    print(f"Error in inverse kinematics: {line}")
                    return None
            
            # 解析 IK Solution 行
            for line in responses:
                if line.startswith("IK Solution:"):
                    parts = line.replace("IK Solution:", "").strip().split()
                    joints = [float(x) for x in parts]
                    if len(joints) == 6:
                        print(f"Inverse kinematics solution: {joints}")
                        if current_joints:
                            print(f"  (using current joints: {current_joints})")
                        return joints
            
            print("No IK solution found in response")
            return None
                
        except Exception as e:
            print(f"Error in inverse kinematics: {e}")
            return None
    
    def move_linear(self, target_pose: dict, velocity: float = 10.0) -> bool:
        """
        直线运动 - 控制末端沿直线运动到目标位姿
        
        Args:
            target_pose: 目标位姿字典
            velocity: 运动速度 (mm/s)，默认 10.0
            
        Returns:
            bool: 运动成功返回 True
        """
        print(f"Linear move to: {target_pose}")
        print("Note: This requires trajectory planning support in firmware")
        
        # 这个功能需要固件支持直线插补
        # 当前可以先使用 set_joints 实现点到点运动
        
        # TODO: 实现直线运动
        return False
    
    def zero(self) -> bool:
        """
        机械臂回零（软复位）
        
        Returns:
            bool: 命令发送成功返回 True
        """
        try:
            responses = self._send_command("zero")
            
            for line in responses:
                if "ERROR" in line:
                    print(f"Error during zeroing: {line}")
                    return False
            
            print("Zero command sent, waiting for completion...")
            time.sleep(5)  # 等待回零完成
            return True
            
        except Exception as e:
            print(f"Error zeroing: {e}")
            return False
    
    def emergency_stop(self):
        """紧急停止"""
        print("EMERGENCY STOP!")
        # 可以通过发送特定命令实现紧急停止
        # 例如禁用远程控制或发送停止命令
        if self.ser and self.ser.is_open:
            self.ser.write(b"remote_disable\n")
    
    def set_gripper(self, angle: float, duration_ms: int = 0) -> bool:
        """
        设置 MG996R 舵机角度（夹爪控制）
        
        Args:
            angle: 目标角度 (0° ~ 180°)
                0°: 完全闭合
                90°: 中间位置
                180°: 完全打开
            duration_ms: 运动时间（毫秒），默认 0 表示立即到达
            
        Returns:
            bool: 命令发送成功返回 True
        """
        if angle < 0 or angle > 180:
            raise ValueError("Gripper angle must be between 0 and 180 degrees")
        
        try:
            # 构建命令
            if duration_ms > 0:
                cmd = f"mg_set {angle:.2f} {duration_ms}"
            else:
                cmd = f"mg_set {angle:.2f}"
            
            responses = self._send_command(cmd)
            
            # 检查是否有错误
            for line in responses:
                if "ERROR" in line:
                    print(f"Error setting gripper: {line}")
                    return False
            
            print(f"Set gripper to {angle:.2f}° (duration: {duration_ms}ms)")
            return True
            
        except Exception as e:
            print(f"Error setting gripper: {e}")
            return False
    
    def get_gripper(self) -> Optional[float]:
        """
        获取 MG996R 舵机当前角度
        
        Returns:
            float: 当前角度值 (0° ~ 180°)，失败返回 None
        """
        try:
            responses = self._send_command("mg_get")
            
            # 解析响应，查找 "Current angle:" 行
            for line in responses:
                if "Current angle:" in line:
                    # 提取角度值
                    parts = line.split("Current angle:")
                    if len(parts) > 1:
                        angle_str = parts[1].strip().split()[0]
                        angle = float(angle_str)
                        print(f"Gripper angle: {angle:.2f}°")
                        return angle
            
            print("Failed to parse gripper angle from response")
            return None
            
        except Exception as e:
            print(f"Error getting gripper angle: {e}")
            return None
    
    def gripper_open(self, duration_ms: int = 1000) -> bool:
        """
        打开夹爪
        
        Args:
            duration_ms: 运动时间（毫秒），默认 1 秒
            
        Returns:
            bool: 命令发送成功返回 True
        """
        return self.set_gripper(180.0, duration_ms)
    
    def gripper_close(self, duration_ms: int = 1000) -> bool:
        """
        闭合夹爪
        
        Args:
            duration_ms: 运动时间（毫秒），默认 1 秒
            
        Returns:
            bool: 命令发送成功返回 True
        """
        return self.set_gripper(0.0, duration_ms)
    
    def gripper_center(self, duration_ms: int = 500) -> bool:
        """
        夹爪回到中间位置
        
        Args:
            duration_ms: 运动时间（毫秒），默认 0.5 秒
            
        Returns:
            bool: 命令发送成功返回 True
        """
        return self.set_gripper(90.0, duration_ms)


def test_robot_arm():
    """测试机械臂控制类"""
    # 创建机械臂实例
    robot = RobotArm(port="COM3", baudrate=115200)
    
    # 连接
    if not robot.connect():
        print("Failed to connect to robot arm")
        return
    
    try:
        # 测试 1：获取当前关节角度
        print("\n=== Test 1: Get Current Joints ===")
        joints = robot.get_joints()
        if joints:
            print(f"Current joints: {joints}")
        
        # 测试 2：设置关节角度
        print("\n=== Test 2: Set Joints ===")
        target_angles = [0, 0, 0, 0, -2.1, 0]
        success = robot.set_joints(target_angles)
        if success:
            print(f"Successfully set joints to {target_angles}")
            time.sleep(2)
            
            # 验证
            joints = robot.get_joints()
            if joints:
                print(f"Verified joints: {joints}")
        
        # 测试 3：获取末端位姿
        print("\n=== Test 3: Get End-Effector Pose ===")
        pose = robot.get_eepose()
        if pose:
            print(f"End-effector pose:")
            print(f"  Position: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f}) mm")
            print(f"  Orientation: (R={pose['roll']:.2f}°, P={pose['pitch']:.2f}°, Y={pose['yaw']:.2f}°)")
        
        # 测试 4：正运动学
        print("\n=== Test 4: Forward Kinematics ===")
        test_angles = [0, 10, -5, 0, 0, 0]
        print(f"Setting joints to: {test_angles}")
        pose = robot.forward_kinematics(test_angles)
        if pose:
            print(f"Result pose: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f}) mm")
        
        # 测试 5：MG996R 舵机控制
        print("\n=== Test 5: MG996R Gripper Control ===")
        
        # 测试 5.1：获取当前角度
        print("Test 5.1: Get current gripper angle")
        gripper_angle = robot.get_gripper()
        if gripper_angle is not None:
            print(f"Current gripper angle: {gripper_angle:.2f}°")
        
        # 测试 5.2：设置角度（立即）
        print("\nTest 5.2: Set gripper to 90° (immediate)")
        robot.set_gripper(90.0)
        time.sleep(1)
        
        # 测试 5.3：设置角度（平滑运动）
        print("\nTest 5.3: Set gripper to 0° (close, 1s)")
        robot.set_gripper(0.0, 1000)
        time.sleep(1.5)
        
        # 测试 5.4：打开夹爪
        print("\nTest 5.4: Open gripper (180°, 1s)")
        robot.gripper_open(1000)
        time.sleep(1.5)
        
        # 测试 5.5：回到中间位置
        print("\nTest 5.5: Move to center position (90°, 0.5s)")
        robot.gripper_center(500)
        time.sleep(1)
        
        # 测试 5.6：验证角度
        print("\nTest 5.6: Verify gripper angle")
        gripper_angle = robot.get_gripper()
        if gripper_angle is not None:
            print(f"Verified gripper angle: {gripper_angle:.2f}°")
        
    finally:
        # 断开连接
        robot.disconnect()


if __name__ == "__main__":
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="Robot Arm Control")
    parser.add_argument("-p", "--port", type=str, default="COM3", help="Serial port")
    parser.add_argument("-b", "--baudrate", type=int, default=115200, help="Baudrate")
    parser.add_argument("-t", "--test", action="store_true", help="Run test")
    
    args = parser.parse_args()
    
    if args.test:
        test_robot_arm()
    else:
        # 交互式控制
        robot = RobotArm(port=args.port, baudrate=args.baudrate)
        
        if robot.connect():
            print("\nRobot arm connected. Commands:")
            print("  get_joints     - Get current joint angles")
            print("  set_joints     - Set joint angles")
            print("  get_eepose     - Get end-effector pose")
            print("  zero           - Zero the robot")
            print("  --- Gripper Control ---")
            print("  get_gripper    - Get gripper angle")
            print("  set_gripper    - Set gripper angle")
            print("  gripper_open   - Open gripper (180°)")
            print("  gripper_close  - Close gripper (0°)")
            print("  gripper_center - Center gripper (90°)")
            print("  quit           - Exit program")
            print()
            
            while True:
                try:
                    cmd = input("> ").strip()
                    
                    if cmd == "quit":
                        break
                    elif cmd == "get_joints":
                        joints = robot.get_joints()
                        if joints:
                            print(f"Joints: {joints}")
                    elif cmd == "set_joints":
                        angles = input("Enter 6 angles (space-separated): ").split()
                        angles = [float(a) for a in angles]
                        robot.set_joints(angles)
                    elif cmd == "get_eepose":
                        pose = robot.get_eepose()
                        if pose:
                            print(f"Pose: {pose}")
                    elif cmd == "zero":
                        robot.zero()
                    elif cmd == "get_gripper":
                        angle = robot.get_gripper()
                        if angle is not None:
                            print(f"Gripper angle: {angle:.2f}°")
                    elif cmd == "set_gripper":
                        angle_input = input("Enter angle (0-180): ").strip()
                        duration_input = input("Enter duration in ms (0 for immediate, default 1000): ").strip()
                        try:
                            angle = float(angle_input)
                            duration = int(duration_input) if duration_input else 1000
                            robot.set_gripper(angle, duration)
                        except ValueError as e:
                            print(f"Invalid input: {e}")
                    elif cmd == "gripper_open":
                        duration_input = input("Enter duration in ms (default 1000): ").strip()
                        duration = int(duration_input) if duration_input else 1000
                        robot.gripper_open(duration)
                    elif cmd == "gripper_close":
                        duration_input = input("Enter duration in ms (default 1000): ").strip()
                        duration = int(duration_input) if duration_input else 1000
                        robot.gripper_close(duration)
                    elif cmd == "gripper_center":
                        duration_input = input("Enter duration in ms (default 500): ").strip()
                        duration = int(duration_input) if duration_input else 500
                        robot.gripper_center(duration)
                    else:
                        print(f"Unknown command: {cmd}")
                        
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"Error: {e}")
            
            robot.disconnect()