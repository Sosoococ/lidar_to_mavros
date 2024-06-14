#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PosePublisherNode : public rclcpp::Node
{
public:
    PosePublisherNode() : Node("lidar_odom_to_mavros")
    {
        this->declare_parameter<std::string>("subscribe_topic", "/odom");
        this->declare_parameter<std::string>("publish_topic", "/pose_stamped");
        this->declare_parameter<double>("publish_rate", 10.0);

        this->get_parameter("subscribe_topic", subscribe_topic_);
        this->get_parameter("publish_topic", publish_topic_);
        this->get_parameter("publish_rate", publish_rate_);

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(publish_topic_, 10);
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            subscribe_topic_, 10, std::bind(&PosePublisherNode::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = msg->pose.pose;
        pose_publisher_->publish(pose_stamped);
        RCLCPP_INFO(this->get_logger(), "Publishing pose: [position: (%.2f, %.2f, %.2f), orientation: (%.2f, %.2f, %.2f, %.2f)]",
                     pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z,
                     pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    std::string subscribe_topic_;
    std::string publish_topic_;
    double publish_rate_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PosePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}