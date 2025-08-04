#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class DiffDriveSim : public rclcpp::Node
{
public:
    DiffDriveSim() : Node("diff_drive_sim_cpp")
    {
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DiffDriveSim::cmdCallback, this, std::placeholders::_1));
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        last_time_ = now();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&DiffDriveSim::update, this));
    }

private:
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_cmd_ = *msg;
    }

    void update()
    {
        auto current_time = now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double vx = current_cmd_.linear.x;
        double wz = current_cmd_.angular.z;
        pose_x_ += vx * dt * std::cos(pose_theta_);
        pose_y_ += vx * dt * std::sin(pose_theta_);
        pose_theta_ += wz * dt;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = pose_x_;
        odom.pose.pose.position.y = pose_y_;
        odom.pose.pose.orientation.z = std::sin(pose_theta_ / 2.0);
        odom.pose.pose.orientation.w = std::cos(pose_theta_ / 2.0);
        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = pose_x_;
        t.transform.translation.y = pose_y_;
        t.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::Twist current_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
    double pose_x_{0.0}, pose_y_{0.0}, pose_theta_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveSim>());
    rclcpp::shutdown();
    return 0;
}