#!/usr/bin/env python3
"""
get_ee_pose.py - Franka FR3末端执行器位姿获取工具

此程序:
1. 获取当前末端执行器(panda_link8)相对于基座(panda_link0)的位姿
2. 以清晰格式输出位置和方向（四元数和欧拉角）
3. 可用于验证手眼系统一致性

用法:
    ros2 run arm_follow_tag get_ee_pose

输出示例:
    === 末端执行器当前位姿 (相对于panda_link0) ===
    位置 (m): x=0.402, y=0.001, z=0.398
    方向 (四元数): qx=0.002, qy=0.099, qz=-0.001, qw=0.995
    方向 (欧拉角): roll=0.2°, pitch=170.1°, yaw=-0.1°
    ===========================================
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation

class EEPoseFetcher(Node):
    def __init__(self):
        super().__init__('ee_pose_fetcher')
        
        # === 配置参数 ===
        self.base_frame = "fr3_link0"    # 基座坐标系
        self.ee_frame = "fr3_link8"      # 末端执行器坐标系
        
        # === TF监听器 ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("等待TF系统...")
        
        # 等待TF系统初始化
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds/1e9 < 5.0:
            if self.tf_buffer.can_transform(
                self.base_frame, 
                self.ee_frame, 
                rclpy.time.Time()
            ):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error("等待TF超时! 请确保机械臂已启动并发布TF")
            raise RuntimeError("TF系统未就绪")
            
        self.get_logger().info("TF系统就绪，正在获取末端位姿...")

    def get_ee_pose(self):
        """获取末端执行器相对于基座的位姿"""
        try:
            # 获取变换
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 提取位置
            position = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            
            # 提取方向 (四元数)
            orientation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            return position, orientation
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF查找失败: {str(e)}")
            return None, None

    def quaternion_to_euler(self, qx, qy, qz, qw):
        """将四元数转换为欧拉角(roll, pitch, yaw)"""
        # 使用scipy进行转换
        r = Rotation.from_quat([qx, qy, qz, qw])
        euler = r.as_euler('xyz', degrees=True)
        return euler[0], euler[1], euler[2]

    def format_pose(self, position, orientation):
        """格式化位姿为可读字符串"""
        # 计算欧拉角
        roll, pitch, yaw = self.quaternion_to_euler(*orientation)
        
        # 格式化输出
        output = []
        output.append("\n=== 末端执行器当前位姿 (相对于{}) ===".format(self.base_frame))
        output.append(f"位置 (m): x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
        output.append(f"方向 (四元数): qx={orientation[0]:.3f}, qy={orientation[1]:.3f}, "
                     f"qz={orientation[2]:.3f}, qw={orientation[3]:.3f}")
        output.append(f"方向 (欧拉角): roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°")
        output.append("===========================================")
        
        return "\n".join(output)

    def get_formatted_pose(self):
        """获取格式化的位姿字符串"""
        position, orientation = self.get_ee_pose()
        if position is None or orientation is None:
            return "无法获取末端位姿"
        return self.format_pose(position, orientation)

def main():
    # 初始化ROS 2
    rclpy.init()
    
    try:
        # 创建节点
        node = EEPoseFetcher()
        
        # 获取并显示位姿
        formatted_pose = node.get_formatted_pose()
        print(formatted_pose)
        
        # 也以JSON格式输出（便于程序使用）
        position, orientation = node.get_ee_pose()
        if position is not None and orientation is not None:
            print("\nJSON格式 (可用于程序输入):")
            print("{")
            print(f'  "position": [{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}],')
            print(f'  "orientation": [{orientation[0]:.6f}, {orientation[1]:.6f}, {orientation[2]:.6f}, {orientation[3]:.6f}]')
            print("}")
        
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"发生错误: {str(e)}")
        return 1
        
    return 0

if __name__ == '__main__':
    exit(main())