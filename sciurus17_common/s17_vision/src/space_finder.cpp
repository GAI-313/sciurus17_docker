#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
//#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// 平面検出に使用する PCL
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/common/centroid.h"  
#include "pcl/common/common.h"    
#include "pcl/segmentation/extract_clusters.h"

// クラス
class SpaceFinder : public rclcpp::Node {
public:
    SpaceFinder() : Node("space_finder") {
        // pointcloud サブスクライバーを作成
        pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/head_camera/depth/color/points", 10, std::bind(&SpaceFinder::pc_callback, this, std::placeholders::_1)
        );
        // tf2::BufferCoreの初期化
        //tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_ = std::shared_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock()));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // 検出した平面部をパブリッシュ
        pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("s17_vision/space/points", 10);
        pc_not_space_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("s17_vision/not_space/points", 10);
        // 検出した物体情報をパブリッシュ
        object_info_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("s17_vision/object_info", 10);
        // ノードインスタンスを取得
        node = std::shared_ptr<rclcpp::Node>(this, std::function<void(rclcpp::Node*)>(nullptr));
        // サービスサーバーを作成
        start_stop_detection_service_ = this->create_service<std_srvs::srv::SetBool>(
            "s17_vision/space_finder", std::bind(&SpaceFinder::startStopDetection, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
private:
    // tf2関連の新しいメンバ変数
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree{new pcl::search::KdTree<pcl::PointXYZRGB>};
    std::vector<geometry_msgs::msg::PoseStamped> detected_objects_list;
    bool detection_active_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_detection_service_;

    void startStopDetection(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response) {
        if (request->data && !detection_active_) {
            // サービスクライアントから true が要求され、かつ平面検出がまだ開始されていない場合
            RCLCPP_INFO(this->get_logger(), "Starting plane detection.");
            detection_active_ = true;
        } else if (!request->data && detection_active_) {
            // サービスクライアントから false が要求され、かつ平面検出が既に開始されている場合
            RCLCPP_INFO(this->get_logger(), "Stopping plane detection.");
            detection_active_ = false;
        }

        // サービスの応答
        response->success = true;
        response->message = "Detection state updated.";
    }

    
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {

        if (detection_active_) {
            try {
                // SpaceFinder 処理
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromROSMsg(*pointcloud_msg, *cloud);
                // RGB値を赤色に設定
                for (auto& point : cloud->points) {
                    point.r = 255;
                    point.g = 0;
                    point.b = 0;
                }
                RCLCPP_INFO(this->get_logger(), "Processing for pointcloud ...");
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

                if (inliers->indices.empty()) {
                    std::cerr << "Plane model not found in the point cloud!" << std::endl;
                    return;
                }
                // 平面以外の点を抽出
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_not_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
                extract.setNegative(true);
                extract.filter(*cloud_not_plane);

                // 物体の中心座標と幅を計算
                if (!cloud_not_plane->empty()) {
                    std::vector<pcl::PointIndices> cluster_indices;
                    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
                    ec.setClusterTolerance(0.02);  // クラスタとみなす点間の最大距離
                    ec.setMinClusterSize(100);     // クラスタの最小サイズ
                    ec.setMaxClusterSize(25000);   // クラスタの最大サイズ
                    ec.setSearchMethod(tree);
                    ec.setInputCloud(cloud_not_plane);
                    ec.extract(cluster_indices);

                    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
                        for (const auto& idx : it->indices) {
                            cloud_cluster->points.push_back(cloud_not_plane->points[idx]);
                        }

                        // 物体の中心座標を計算
                        Eigen::Vector4f centroid;
                        pcl::compute3DCentroid(*cloud_cluster, centroid);

                        // 物体の幅を計算
                        pcl::PointXYZRGB min_point, max_point;
                        pcl::getMinMax3D(*cloud_cluster, min_point, max_point);
                        double width = max_point.x - min_point.x;

                        // 物体の中心座標と幅をメッセージに格納
                        geometry_msgs::msg::PoseStamped object_info_msg;
                        object_info_msg.header = pointcloud_msg->header;
                        object_info_msg.pose.position.x = centroid[0];
                        object_info_msg.pose.position.y = centroid[1];
                        object_info_msg.pose.position.z = centroid[2];
                        object_info_msg.pose.orientation.w = centroid[3];
                        //object_info_msg.pose.orientation.w = width;

                        convertToBaseLinkFrame(object_info_msg);

                        // リストに追加
                        detected_objects_list.push_back(object_info_msg);
                    }
                    // 検出した物体情報を一度にパブリッシュ
                    publish_detected_objects();
                }

                // 新しいPointCloud2メッセージを作成
                sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                pcl::toROSMsg(*cloud_plane, *cloud_msg);

                sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_not_space = std::make_shared<sensor_msgs::msg::PointCloud2>();
                pcl::toROSMsg(*cloud_not_plane, *cloud_msg_not_space);

                // 新しいトピックに発行
                pc_pub->publish(*cloud_msg);
                pc_not_space_pub->publish(*cloud_msg_not_space);
                // SpaceFinder 終了
            } catch (std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "PointCloud is Empty.");
            }
        } // end
    }

    void convertToBaseLinkFrame(geometry_msgs::msg::PoseStamped& object_info_msg) {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped;
            // ターゲットフレーム(base_link)からの変換を取得
            transform_stamped = tf_buffer_->lookupTransform("base_link", object_info_msg.header.frame_id, rclcpp::Time(0));
            
            // 物体の中心座標をbase_link座標系に変換
            //tf2::doTransform(object_info_msg.pose, object_info_msg.pose, transform_stamped);
            tf2::doTransform(object_info_msg.pose.position, object_info_msg.pose.position, transform_stamped);
            tf2::doTransform(object_info_msg.pose.orientation, object_info_msg.pose.orientation, transform_stamped);

            // ヘッダー情報も更新
            object_info_msg.header.frame_id = "base_link";
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }
    }
    
    void publish_detected_objects() {
        if (!detected_objects_list.empty()) {
            for (const auto& object_info_msg : detected_objects_list) {
                object_info_pub->publish(object_info_msg);
            }
            detected_objects_list.clear();
        }
    }
    
    // メンバ変数を登録
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_not_space_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_info_pub;
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
