#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

class VSlamToPx4Node : public rclcpp::Node
{
public:
    VSlamToPx4Node() : Node("vslam_to_px4_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("vslam_odom_topic", "/camera/odom/sample");
        this->declare_parameter<std::string>("px4_odom_topic", "/fmu/out/vehicle_odometry");
        this->declare_parameter<bool>("use_pose_topic", false);
        this->declare_parameter<std::string>("vslam_pose_topic", "/camera/pose");
        
        // 获取参数
        std::string vslam_odom_topic = this->get_parameter("vslam_odom_topic").as_string();
        std::string px4_odom_topic = this->get_parameter("px4_odom_topic").as_string();
        bool use_pose_topic = this->get_parameter("use_pose_topic").as_bool();
        std::string vslam_pose_topic = this->get_parameter("vslam_pose_topic").as_string();
        
        RCLCPP_INFO(this->get_logger(), "VSLAM to PX4 node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", vslam_odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", px4_odom_topic.c_str());
        
        // 创建发布者
        px4_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            px4_odom_topic, 10);
        
        // 创建订阅者 - 根据参数选择订阅里程计或位姿
        if (use_pose_topic) {
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                vslam_pose_topic, 10,
                std::bind(&VSlamToPx4Node::poseCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Also subscribing to pose: %s", vslam_pose_topic.c_str());
        } else {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                vslam_odom_topic, 10,
                std::bind(&VSlamToPx4Node::odomCallback, this, std::placeholders::_1));
        }
        
        // 初始化时间戳
        last_timestamp_ = 0;
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto px4_odom = convertOdometryToPx4(*msg);
        px4_odom_pub_->publish(px4_odom);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Published odometry - Position: [%.3f, %.3f, %.3f], Orientation: [%.3f, %.3f, %.3f, %.3f]",
            px4_odom.position[0], px4_odom.position[1], px4_odom.position[2],
            px4_odom.q[0], px4_odom.q[1], px4_odom.q[2], px4_odom.q[3]);
    }
    
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        auto px4_odom = convertPoseToPx4(*msg);
        px4_odom_pub_->publish(px4_odom);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Published pose - Position: [%.3f, %.3f, %.3f], Orientation: [%.3f, %.3f, %.3f, %.3f]",
            px4_odom.position[0], px4_odom.position[1], px4_odom.position[2],
            px4_odom.q[0], px4_odom.q[1], px4_odom.q[2], px4_odom.q[3]);
    }

    px4_msgs::msg::VehicleOdometry convertOdometryToPx4(const nav_msgs::msg::Odometry& odom_msg)
    {
        px4_msgs::msg::VehicleOdometry px4_odom;
        
        // 时间戳转换
        uint64_t timestamp = rclcpp::Time(odom_msg.header.stamp).nanoseconds() / 1000;
        px4_odom.timestamp = timestamp;
        px4_odom.timestamp_sample = timestamp;
        
        // 坐标系转换：ROS (ENU) 到 PX4 (NED)
        // Position: ENU -> NED (x->y, y->x, z->-z)
        px4_odom.position[0] = odom_msg.pose.pose.position.y;  // North
        px4_odom.position[1] = odom_msg.pose.pose.position.x;  // East  
        px4_odom.position[2] = -odom_msg.pose.pose.position.z; // Down
        
        // Velocity: ENU -> NED
        px4_odom.velocity[0] = odom_msg.twist.twist.linear.y;  // North
        px4_odom.velocity[1] = odom_msg.twist.twist.linear.x;  // East
        px4_odom.velocity[2] = -odom_msg.twist.twist.linear.z; // Down
        
        // Angular velocity: ENU -> NED
        px4_odom.angular_velocity[0] = odom_msg.twist.twist.angular.y;
        px4_odom.angular_velocity[1] = odom_msg.twist.twist.angular.x;
        px4_odom.angular_velocity[2] = -odom_msg.twist.twist.angular.z;
        
        // 四元数转换：ROS (ENU) 到 PX4 (NED)
        tf2::Quaternion ros_quat;
        tf2::fromMsg(odom_msg.pose.pose.orientation, ros_quat);
        
        // ENU to NED 旋转: 绕Z轴旋转90度，然后绕X轴旋转180度
        tf2::Quaternion enu_to_ned;
        enu_to_ned.setRPY(M_PI, 0, M_PI_2);
        tf2::Quaternion ned_quat = enu_to_ned * ros_quat;
        ned_quat.normalize();
        
        // PX4使用 [w, x, y, z] 顺序
        px4_odom.q[0] = ned_quat.w();
        px4_odom.q[1] = ned_quat.x();
        px4_odom.q[2] = ned_quat.y();
        px4_odom.q[3] = ned_quat.z();
        
        // 位置协方差转换 (6x6 -> 3x3)
        if (odom_msg.pose.covariance[0] > 0) {
            px4_odom.position_variance[0] = odom_msg.pose.covariance[7];  // y -> North
            px4_odom.position_variance[1] = odom_msg.pose.covariance[0];  // x -> East
            px4_odom.position_variance[2] = odom_msg.pose.covariance[14]; // z -> Down
        } else {
            // 默认协方差
            px4_odom.position_variance[0] = 0.1f;
            px4_odom.position_variance[1] = 0.1f;
            px4_odom.position_variance[2] = 0.1f;
        }
        
        // 姿态协方差转换
        if (odom_msg.pose.covariance[21] > 0) {
            px4_odom.orientation_variance[0] = odom_msg.pose.covariance[28]; // yaw -> roll
            px4_odom.orientation_variance[1] = odom_msg.pose.covariance[21]; // roll -> pitch
            px4_odom.orientation_variance[2] = odom_msg.pose.covariance[35]; // pitch -> yaw
        } else {
            // 默认协方差
            px4_odom.orientation_variance[0] = 0.05f;
            px4_odom.orientation_variance[1] = 0.05f;
            px4_odom.orientation_variance[2] = 0.05f;
        }
        
        // 速度协方差转换
        if (odom_msg.twist.covariance[0] > 0) {
            px4_odom.velocity_variance[0] = odom_msg.twist.covariance[7];  // y -> North
            px4_odom.velocity_variance[1] = odom_msg.twist.covariance[0];  // x -> East
            px4_odom.velocity_variance[2] = odom_msg.twist.covariance[14]; // z -> Down
        } else {
            // 默认协方差
            px4_odom.velocity_variance[0] = 0.1f;
            px4_odom.velocity_variance[1] = 0.1f;
            px4_odom.velocity_variance[2] = 0.1f;
        }
        
        // 设置质量分数 (0-100, 100表示最高质量)
        px4_odom.quality = calculateQualityScore(odom_msg);
        
        // 重置计数器
        px4_odom.reset_counter = 0;
        
                // 设置坐标系类型
        px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        
        return px4_odom;
    }
    
    px4_msgs::msg::VehicleOdometry convertPoseToPx4(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
    {
        px4_msgs::msg::VehicleOdometry px4_odom;
        
        // 时间戳转换
        uint64_t timestamp = rclcpp::Time(pose_msg.header.stamp).nanoseconds() / 1000;
        px4_odom.timestamp = timestamp;
        px4_odom.timestamp_sample = timestamp;
        
        // 坐标系转换：ROS (ENU) 到 PX4 (NED)
        px4_odom.position[0] = pose_msg.pose.pose.position.y;  // North
        px4_odom.position[1] = pose_msg.pose.pose.position.x;  // East  
        px4_odom.position[2] = -pose_msg.pose.pose.position.z; // Down
        
        // 速度设为0（位姿消息不包含速度信息）
        px4_odom.velocity[0] = 0.0f;
        px4_odom.velocity[1] = 0.0f;
        px4_odom.velocity[2] = 0.0f;
        
        // 角速度设为0
        px4_odom.angular_velocity[0] = 0.0f;
        px4_odom.angular_velocity[1] = 0.0f;
        px4_odom.angular_velocity[2] = 0.0f;
        
        // 四元数转换
        tf2::Quaternion ros_quat;
        tf2::fromMsg(pose_msg.pose.pose.orientation, ros_quat);
        
        tf2::Quaternion enu_to_ned;
        enu_to_ned.setRPY(M_PI, 0, M_PI_2);
        tf2::Quaternion ned_quat = enu_to_ned * ros_quat;
        ned_quat.normalize();
        
        px4_odom.q[0] = ned_quat.w();
        px4_odom.q[1] = ned_quat.x();
        px4_odom.q[2] = ned_quat.y();
        px4_odom.q[3] = ned_quat.z();
        
        // 协方差设置
        if (pose_msg.pose.covariance[0] > 0) {
            px4_odom.position_variance[0] = pose_msg.pose.covariance[7];  
            px4_odom.position_variance[1] = pose_msg.pose.covariance[0];  
            px4_odom.position_variance[2] = pose_msg.pose.covariance[14]; 
        } else {
            px4_odom.position_variance[0] = 0.1f;
            px4_odom.position_variance[1] = 0.1f;
            px4_odom.position_variance[2] = 0.1f;
        }
        
        if (pose_msg.pose.covariance[21] > 0) {
            px4_odom.orientation_variance[0] = pose_msg.pose.covariance[28];
            px4_odom.orientation_variance[1] = pose_msg.pose.covariance[21];
            px4_odom.orientation_variance[2] = pose_msg.pose.covariance[35];
        } else {
            px4_odom.orientation_variance[0] = 0.05f;
            px4_odom.orientation_variance[1] = 0.05f;
            px4_odom.orientation_variance[2] = 0.05f;
        }
        
        // 速度协方差（较大，因为没有速度信息）
        px4_odom.velocity_variance[0] = 1.0f;
        px4_odom.velocity_variance[1] = 1.0f;
        px4_odom.velocity_variance[2] = 1.0f;
        
        px4_odom.quality = 50; // 位姿质量分数设为中等
        px4_odom.reset_counter = 0;
        px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        
        return px4_odom;
    }
    
    uint8_t calculateQualityScore(const nav_msgs::msg::Odometry& odom_msg)
    {
        // 基于协方差计算质量分数
        double pos_var = (odom_msg.pose.covariance[0] + odom_msg.pose.covariance[7] + odom_msg.pose.covariance[14]) / 3.0;
        double ori_var = (odom_msg.pose.covariance[21] + odom_msg.pose.covariance[28] + odom_msg.pose.covariance[35]) / 3.0;
        
        // 质量分数计算（协方差越小，质量越高）
        uint8_t quality = 100;
        if (pos_var > 0.01) quality -= 20;
        if (pos_var > 0.1) quality -= 30;
        if (ori_var > 0.01) quality -= 20;
        if (ori_var > 0.1) quality -= 30;
        
        return std::max(1, static_cast<int>(quality));
    }

    // 成员变量
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_pub_;
    uint64_t last_timestamp_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VSlamToPx4Node>();
    
    RCLCPP_INFO(node->get_logger(), "VSLAM to PX4 conversion node is running...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
