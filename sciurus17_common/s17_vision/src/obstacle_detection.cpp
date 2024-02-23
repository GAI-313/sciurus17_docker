#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class ObstacleDetection : public rclcpp::Node{
public:
    ObstacleDetection() : Node("obstacle_detection") {
        RCLCPP_INFO(this->get_logger(), "obstacle detection start");

        // PointCloud サブスクライバー
        p_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/head_camera/depth/color/points", 10,
            std::bind(&ObstacleDetection::pointcloud_callback, this, std::placeholders::_1)
        );
        // MarkerArray パブリッシャー
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/obstacle_markers", 10
        );
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message");

        // PointCloud2 メッセージから x, y, z 座標情報を取得
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        // MarkerArray を作成
    }
    // スコープ
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr p_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
