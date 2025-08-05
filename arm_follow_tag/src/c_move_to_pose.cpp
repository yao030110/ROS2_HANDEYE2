/*从tf树中获取base-fr3_link8-camera_link-tag的tf关系，可以直接读其中的位姿
------NS_1/joint_impedance_with_ik_controller/target_pose-------
 发送数据给这个topic可以直接控制机械臂移动，但是幅度不能太大，3cm左右把
------/NS_1/franka_robot_state_broadcaster/current_pose-----
读这个topic可以直接获取link8到base的tf，这里用作跟期望ee位姿作差值进行小幅移动
手眼标定的关系由easy_handeye2获取
读取tag位姿->第一次读取的为后续使用的相对位姿->更新tag位姿->比较相对位姿的差值，通过运算
算出追踪的target_pose->限速->发布
*/
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <mutex>
#include <cmath>
#include <functional>  // 添加这个头文件
#include <tf2/LinearMath/Quaternion.h> 
// 添加这行以使用 _1 占位符
using std::placeholders::_1;
class TagFollower : public rclcpp::Node
{
public:
  TagFollower() : Node("tag_follower"), 
                  has_current_pose_(false),
                  first_detection_(true)
                  
  {
    // TF 缓冲区和监听器
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅 AprilTag 位姿
    tag_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aruco_single/pose", 10,
      std::bind(&TagFollower::tag_callback, this, _1));
    // 订阅当前末端位姿
    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/NS_1/franka_robot_state_broadcaster/current_pose", 10,
      std::bind(&TagFollower::current_pose_callback, this, _1));
    // 发布目标末端位姿
    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/NS_1/joint_impedance_with_ik_controller/target_pose", 10); //"/NS_1/joint_impedance_with_ik_controller/target_pose"

    // 100Hz 发布频率
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TagFollower::publish_target, this));

    RCLCPP_INFO(this->get_logger(), "Tag follower active. Move arm to desired position and detect tag.");
  }

private:
  void current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = *msg;
    has_current_pose_ = true;
  }
  void tag_callback(const geometry_msgs::msg::PoseStamped::SharedPtr tag_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    try {
      // 1. 计算 ee -> tag 的变换
      tf2::Transform camera_to_tag, ee_to_camera, ee_to_tag;
      tf2::fromMsg(tag_msg->pose, camera_to_tag);
      tf2::fromMsg(ee_to_camera_tf_.transform, ee_to_camera);
      ee_to_tag = ee_to_camera * camera_to_tag;

      if (first_detection_) {
        // 2. 首次检测：设定期望关系
        desired_ee_to_tag_ = ee_to_tag;
        first_detection_ = false;
        RCLCPP_INFO(this->get_logger(), "Initial pose set! Following tag now.");
        return;
      }

      // 3. 后续检测：计算目标 ee 位姿 (base 坐标系)
      auto base_to_tag = tf_buffer_->lookupTransform("base", "aruco_marker_frame", tf2::TimePointZero);
      tf2::Transform base_to_tag_tf, base_to_ee_target;
      tf2::fromMsg(base_to_tag.transform, base_to_tag_tf);
      
      base_to_ee_target = base_to_tag_tf * desired_ee_to_tag_.inverse();
      
      // 4. 准备发布
      target_pose_.header.frame_id = "base";
      target_pose_.header.stamp = this->now();
      tf2::toMsg(base_to_ee_target, target_pose_.pose);
      has_target_ = true;

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_ONCE(this->get_logger(), "TF error: %s", ex.what());
    }
  }

  void publish_target()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 检查是否有有效的当前位姿和目标位姿
    if (!has_current_pose_ || !has_target_) {
      return;
    }

    // 1. 创建独立的新消息（不直接使用类变量）
    auto target_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    target_msg->header.frame_id = "base";
    target_msg->header.stamp = this->now();

    // 2. 获取当前位姿和目标位姿的TF表示
    tf2::Transform current_tf, target_tf;
    tf2::fromMsg(current_pose_.pose, current_tf);
    tf2::fromMsg(target_pose_.pose, target_tf);

    // 3. 计算位置差异
    tf2::Vector3 pos_diff = target_tf.getOrigin() - current_tf.getOrigin();
    double pos_distance = pos_diff.length();

    // 4. 位置平滑处理
    if (pos_distance > 0.03) {
      // 缩放到0.1的距离
      pos_diff = pos_diff * (0.02 / pos_distance);
    }
    
    // 5. 计算目标位置（当前位置 + 调整后的差异）
    tf2::Vector3 new_position = current_tf.getOrigin() + pos_diff;

    // 6. 方向处理 - 转换为欧拉角进行比较
    double cr, cp, cy, cr2, cp2, cy2;
    tf2::Matrix3x3(current_tf.getRotation()).getRPY(cr, cp, cy);
    tf2::Matrix3x3(target_tf.getRotation()).getRPY(cr2, cp2, cy2);
    
    // 计算角度差异（考虑角度周期性）
    double dr = normalize_angle(cr2 - cr);
    double dp = normalize_angle(cp2 - cp);
    double dy = normalize_angle(cy2 - cy);
    
    // 角度平滑处理
    const double ANGLE_THRESHOLD = 0.02; // 弧度，约5.7度
    if (std::abs(dr) > ANGLE_THRESHOLD) {
      dr = (dr > 0) ? ANGLE_THRESHOLD : -ANGLE_THRESHOLD;
    }
    if (std::abs(dp) > ANGLE_THRESHOLD) {
      dp = (dp > 0) ? ANGLE_THRESHOLD : -ANGLE_THRESHOLD;
    }
    if (std::abs(dy) > ANGLE_THRESHOLD) {
      dy = (dy > 0) ? ANGLE_THRESHOLD : -ANGLE_THRESHOLD;
    }
    
    // 计算新角度
    double new_r = cr + dr;
    double new_p = cp + dp;
    double new_y = cy + dy;
    
    // 7. 创建新的变换
    tf2::Transform new_tf;
    new_tf.setOrigin(new_position);
    // ✅ 修复：正确设置Quaternion
    tf2::Quaternion q;
    q.setRPY(new_r, new_p, new_y);
    new_tf.setRotation(q);
    
    // 8. 转换为消息并发布
    tf2::toMsg(new_tf, target_msg->pose);
    target_pub_->publish(std::move(target_msg));
  }
  
  // 辅助函数：规范化角度到[-π, π]
  double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
  }

  // 手眼标定参数 (ee -> camera)
  geometry_msgs::msg::TransformStamped ee_to_camera_tf_ = [] {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "fr3_link8";
    tf.child_frame_id = "camera_link";
    tf.transform.translation.x = -0.0362;
    tf.transform.translation.y = -0.1013;
    tf.transform.translation.z = 0.0352;
    tf.transform.rotation.x = -0.2693;
    tf.transform.rotation.y = -0.1893;
    tf.transform.rotation.z = -0.1594;
    tf.transform.rotation.w = 0.9307;
    return tf;
  }();

  // 核心变量
  tf2::Transform desired_ee_to_tag_;
  geometry_msgs::msg::PoseStamped target_pose_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_target_ = false;
  bool has_current_pose_ = false;
  bool first_detection_;

  // ROS 2 组件
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // TF 组件
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // 线程安全
  std::mutex mutex_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TagFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}