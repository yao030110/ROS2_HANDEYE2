#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
import time

def main():
    rclpy.init()
    
    # 创建最简节点
    node = Node('simple_pose_mover')
    
    # 创建 MoveGroup Action 客户端
    action_client = ActionClient(node, MoveGroup, '/move_action')
    
    # 等待服务器 (等待更长时间)
    if not action_client.wait_for_server(timeout_sec=20.0):
        node.get_logger().error("MoveGroup action server not available after 20 seconds")
        return
    
    node.get_logger().info("Connected to MoveGroup action server")
    
    try:
        # 等待系统完全初始化
        time.sleep(3.0)
        
        # 创建目标位姿 - 使用绝对安全的位姿
        # 这个位姿非常靠近基座，肯定在工作空间内
        target_pose = PoseStamped()
        target_pose.header.frame_id = "fr3_link0"
        target_pose.header.stamp = node.get_clock().now().to_msg()
        target_pose.pose.position.x = 0.2    # 非常保守的位置
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.2
        target_pose.pose.orientation.w = 1.0  # 单位四元数
        
        # 创建运动规划请求
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "fr3_manipulator"
        goal_msg.request.allowed_planning_time = 30.0  # 充足的规划时间
        
        # 仅使用位置约束 (关键!)
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = "fr3_link8"
        pos_constraint.weight = 1.0
        
        # 创建宽松的球体约束 (2cm 容差)
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]  # 2cm 容差
        
        sphere_pose = Pose()
        sphere_pose.position = target_pose.pose.position
        bounding_volume.primitives = [sphere]
        bounding_volume.primitive_poses = [sphere_pose]
        pos_constraint.constraint_region = bounding_volume
        
        # 仅设置位置约束 (移除方向约束 - 这是关键!)
        pose_constraints = Constraints()
        pose_constraints.position_constraints = [pos_constraint]
        goal_msg.request.goal_constraints = [pose_constraints]
        
        # 发送目标
        node.get_logger().info("Sending goal to MoveGroup")
        send_goal_future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            node.get_logger().error("Goal rejected by MoveGroup")
            return
        
        node.get_logger().info("Goal accepted by MoveGroup")
        
        # 等待结果
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, get_result_future)
        result = get_result_future.result().result
        
        # 检查结果
        if result.error_code.val == 1:  # SUCCESS
            node.get_logger().info("✅ SUCCESS: Successfully reached the target pose")
        else:
            node.get_logger().error(f"❌ FAILED: Planning failed with error code: {result.error_code.val}")
            node.get_logger().info("This is likely because:")
            node.get_logger().info("- The system isn't fully initialized (try increasing sleep time)")
            node.get_logger().info("- Controllers aren't active (check with 'ros2 control list_controllers')")
            node.get_logger().info("- MoveGroup isn't properly configured")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()