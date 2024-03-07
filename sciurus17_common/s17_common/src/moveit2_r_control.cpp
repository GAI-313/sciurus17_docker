#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "s17_msgs/srv/set_position.hpp"

const std::string PLANNING_GROUP = "r_arm_group";

class RightArmController
{
public:
    RightArmController() : node_(rclcpp::Node::make_shared("moveit2_right_control"))
    {
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);

        // サブスクライバーの設定
        subscriber_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/s17_common/moveit2_control/r_arm/pose",
            10,
            std::bind(&RightArmController::poseCallback, this, std::placeholders::_1));

        // サービスサーバーの設定
        service_ = node_->create_service<s17_msgs::srv::SetPosition>(
            "/s17_common/r_arm/set_position",
            std::bind(&RightArmController::setPositionCallback, this, std::placeholders::_1, std::placeholders::_2));

        // executor の初期化
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
    }

    void moveLeftArm(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // ここで move_group_ を使用して左腕を指定された位置・姿勢に移動
        move_group_->setPoseTarget(*msg);
        move_group_->move();
    }

    // サブスクライバーのコールバック関数
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Received pose: x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
                    msg->position.x, msg->position.y, msg->position.z,
                    msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

        moveLeftArm(msg);
    }

    // サービスサーバーのコールバック関数
    void setPositionCallback(
        const std::shared_ptr<s17_msgs::srv::SetPosition::Request> request,
        std::shared_ptr<s17_msgs::srv::SetPosition::Response> response)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = request->x;
        pose.position.y = request->y;
        pose.position.z = request->z;
        pose.orientation.x = request->ox;
        pose.orientation.y = request->oy;
        pose.orientation.z = request->oz;
        pose.orientation.w = request->ow;

        moveLeftArm(std::make_shared<geometry_msgs::msg::Pose>(pose));
        response->success = true;
    }

    void spin()
    {
        executor_->spin();
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
    rclcpp::Service<s17_msgs::srv::SetPosition>::SharedPtr service_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RightArmController right_arm_controller;
    right_arm_controller.spin();
    rclcpp::shutdown();
    return 0;
}
