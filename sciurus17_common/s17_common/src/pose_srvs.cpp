#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/joint_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_srvs/srv/set_bool.hpp"

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller") {
        RCLCPP_INFO(this->get_logger(), "Start init_pose service");
        // パブリッシャーを作成
        l_traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "left_arm_controller/joint_trajectory", 10);
        r_traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "right_arm_controller/joint_trajectory", 10);
        n_traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "neck_controller/joint_trajectory", 10);
        w_traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "waist_yaw_controller/joint_trajectory", 10);

        // サービスを作成
        init_pose_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/s17_common/init_pose", std::bind(&RobotController::handleInitPoseService, this, std::placeholders::_1, std::placeholders::_2));

        // 目標姿勢の初期化
        l_pose = {
            1.57,  // l_arm_joint1
            1.57,  // l_arm_joint2
            0.0,   // l_arm_joint3
            -2.36, // l_arm_joint4
            0.0,   // l_arm_joint5
            0.75,  // l_arm_joint6
            0.0,   // l_arm_joint7
        };

        r_pose = {
            -1.57,  // r_arm_joint1
            -1.57,  // r_arm_joint2
            0.0,    // r_arm_joint3
            2.36,   // r_arm_joint4
            0.0,    // r_arm_joint5
            -0.75,  // r_arm_joint6
            0.0,    // r_arm_joint7
        };

        n_pose = {0.0, 0.0};
        w_pose = {0.0};
    }

private:
    void handleInitPoseService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data) {
            // 初期姿勢を目標とする
            RCLCPP_INFO(this->get_logger(), "Go to init ...");

            trajectory_msgs::msg::JointTrajectory l_traj;
            trajectory_msgs::msg::JointTrajectory r_traj;
            trajectory_msgs::msg::JointTrajectory n_traj;
            trajectory_msgs::msg::JointTrajectory w_traj;

            l_traj.joint_names = l_joints;
            l_traj.points.resize(1);
            l_traj.points[0].positions = l_pose;

            r_traj.joint_names = r_joints;
            r_traj.points.resize(1);
            r_traj.points[0].positions = r_pose;

            n_traj.joint_names = n_joints;
            n_traj.points.resize(1);
            n_traj.points[0].positions = n_pose;

            w_traj.joint_names = w_joints;
            w_traj.points.resize(1);
            w_traj.points[0].positions = w_pose;

            // 到達時間を設定（例: 3秒で目標ポイントへ到達）
            l_traj.points[0].time_from_start = rclcpp::Duration(2, 0);
            r_traj.points[0].time_from_start = rclcpp::Duration(2, 0);
            n_traj.points[0].time_from_start = rclcpp::Duration(2, 0);
            w_traj.points[0].time_from_start = rclcpp::Duration(2, 0);

            // 制御信号としてジョイント軌道をパブリッシュ
            l_traj_pub->publish(l_traj);
            r_traj_pub->publish(r_traj);
            n_traj_pub->publish(n_traj);
            w_traj_pub->publish(w_traj);

            response->success = true;
            response->message = "Init pose executed.";
        } else {
            response->success = false;
            response->message = "Invalid request. Data should be true.";
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr l_traj_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr r_traj_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr n_traj_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr w_traj_pub;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr init_pose_service_;
    sensor_msgs::msg::JointState::SharedPtr initial_joint_state_;
    std::vector<double> l_pose;
    std::vector<double> r_pose;
    std::vector<double> n_pose;
    std::vector<double> w_pose;
    std::vector<std::string> l_joints{"l_arm_joint1", "l_arm_joint2", "l_arm_joint3", "l_arm_joint4", "l_arm_joint5", "l_arm_joint6", "l_arm_joint7"};
    std::vector<std::string> r_joints{"r_arm_joint1", "r_arm_joint2", "r_arm_joint3", "r_arm_joint4", "r_arm_joint5", "r_arm_joint6", "r_arm_joint7"};
    std::vector<std::string> n_joints{"neck_yaw_joint", "neck_pitch_joint"};
    std::vector<std::string> w_joints{"waist_yaw_joint"};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
