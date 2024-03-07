#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/joint_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "s17_common/srv/set_pose.hpp"

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("set_pose") {
        RCLCPP_INFO(this->get_logger(), "Start set_pose service");
        // ジョイント状態をサブスクライブ
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&RobotController::jointStateCallback, this, std::placeholders::_1));
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
        init_pose_service_ = this->create_service<s17_common::srv::SetPose>(
            "/s17_common/set_pose", std::bind(&RobotController::handleInitPoseService, this, std::placeholders::_1, std::placeholders::_2));

        // 初期ジョイント状態を取得し、joint_map_ を初期化
        initial_joint_state_ = getCurrentJointState();
        initializeJointMap();

        // 目標姿勢の初期化
        l_pose = {
            0.0,  // l_arm_joint1
            0.0,  // l_arm_joint2
            0.0,   // l_arm_joint3
            0.0, // l_arm_joint4
            0.0,   // l_arm_joint5
            0.0,  // l_arm_joint6
            0.0,   // l_arm_joint7
        };

        r_pose = {
            0.0,  // r_arm_joint1
            0.0,  // r_arm_joint2
            0.0,    // r_arm_joint3
            0.0,   // r_arm_joint4
            0.0,    // r_arm_joint5
            0.0,  // r_arm_joint6
            0.0,    // r_arm_joint7
        };

        n_pose = {0.0, 0.0};
        w_pose = {0.0};
    }

private:
    void handleInitPoseService(const std::shared_ptr<s17_common::srv::SetPose::Request> request,
                               std::shared_ptr<s17_common::srv::SetPose::Response> response)
    {
         if (!request->pose_names.empty() && !request->pose_values.empty()) {
            // ジョイント名リストとジョイント角度リストが空でない場合の処理
            RCLCPP_INFO(this->get_logger(), "Set pose ...");

            // サービスから得られたジョイント名と角度を取得
            const auto& pose_names = request->pose_names;
            const auto& pose_values = request->pose_values;

            // ジョイント名と角度の対応を確認し、存在すれば角度を変更
            for (size_t i = 0; i < pose_names.size(); ++i) {
                if (joint_map_.find(pose_names[i]) != joint_map_.end()) {
                    // ジョイントが存在する場合、角度を変更
                    joint_map_[pose_names[i]] = pose_values[i];
                    // ここで実際の制御信号のパブリッシュなどの処理を追加する
                } else {
                    RCLCPP_WARN(this->get_logger(), "Joint '%s' not found.", pose_names[i].c_str());
                }
            }
            
            response->success = true;
            response->result_message = "pose executed.";
        } else {
            response->success = false;
            response->result_message = "Invalid request. Data should be true.";
        }
    }

    // 初期ジョイント状態を取得する関数
    sensor_msgs::msg::JointState::SharedPtr getCurrentJointState() {
        // 適切な方法で現在のジョイント状態を取得する実装
        // 例: サブスクライバーを用いて現在のジョイント状態を取得するなど
    }
    
    // joint_map_ を初期化する関数
    void initializeJointMap() {
        joint_map_.clear();
        for (size_t i = 0; i < initial_joint_state_->name.size(); ++i) {
            joint_map_[initial_joint_state_->name[i]] = initial_joint_state_->position[i];
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr l_traj_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr r_traj_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr n_traj_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr w_traj_pub;
    rclcpp::Service<s17_common::srv::SetPose>::SharedPtr init_pose_service_;
    sensor_msgs::msg::JointState::SharedPtr initial_joint_state_;
    std::vector<double> l_pose;
    std::vector<double> r_pose;
    std::vector<double> n_pose;
    std::vector<double> w_pose;
    std::vector<std::string> l_joints{"l_arm_joint1", "l_arm_joint2", "l_arm_joint3", "l_arm_joint4", "l_arm_joint5", "l_arm_joint6", "l_arm_joint7"};
    std::vector<std::string> r_joints{"r_arm_joint1", "r_arm_joint2", "r_arm_joint3", "r_arm_joint4", "r_arm_joint5", "r_arm_joint6", "r_arm_joint7"};
    std::vector<std::string> n_joints{"neck_yaw_joint", "neck_pitch_joint"};
    std::vector<std::string> w_joints{"waist_yaw_joint"};
    std::map<std::string, double> joint_map_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
