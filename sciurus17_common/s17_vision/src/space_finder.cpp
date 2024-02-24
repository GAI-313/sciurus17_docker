#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// 平面検出に使用する PCL
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/visualization/cloud_viewer.h"

// クラス
class SpaceFinder : public rclcpp::Node {
public:
    SpaceFinder() : Node("space_finder") {
        // pointcloud サブスクライバーを作成
        pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/head_camera/depth/color/points", 10, std::bind(&SpaceFinder::pc_callback, this, std::placeholders::_1)
        );
        // 検出した平面部をパブリッシュ
        pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("s17_vision/space/points", 10);
        pc_not_space_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("s17_vision/not_space/points", 10);
        // ノードインスタンスを取得
        node = std::shared_ptr<rclcpp::Node>(this, std::function<void(rclcpp::Node*)>(nullptr));
    }
private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
        RCLCPP_INFO(this->get_logger(), "Processing for pointcloud ...");

        // SpaceFinder 処理
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*pointcloud_msg, *cloud);
        // RGB値を赤色に設定
        for (auto& point : cloud->points) {
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }
        // 平面検出
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        // 平面に該当する点を抽出
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        // 平面以外の点を抽出
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_not_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setNegative(true);
        extract.filter(*cloud_not_plane);

        // 新しいPointCloud2メッセージを作成
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud_plane, *cloud_msg);

        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_not_space = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud_not_plane, *cloud_msg_not_space);

        // 新しいトピックに発行
        pc_pub->publish(*cloud_msg);
        pc_not_space_pub->publish(*cloud_msg_not_space);
        // SpaceFinder 終了
    }
    // メンバ変数を登録
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_not_space_pub;
    std::shared_ptr<rclcpp::Node> node;
};

// ここでノードを初期化、実行
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // Node 宣言とインスタンス化
    auto node = std::make_shared<SpaceFinder>();
    // node をスピン
    rclcpp::spin(node);
    // 終了
    rclcpp::shutdown();
    return 0;
}
