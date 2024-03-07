import rclpy
from sensor_msgs.msg import PointCloud2
import numpy as np

def callback(msg):
    # PointCloud2メッセージからデータを取得
    data = msg.data

    # NumPy配列に変換
    buffer = np.frombuffer(data, dtype=np.uint8)
    
    # ポイントクラウドのデータ型（float32）に変換
    points = buffer.astype(np.float32).reshape(-1, 4)

    # 点群の中心座標を計算
    center = np.mean(points[:, :3], axis=0)

    print("PointCloud Center: ", center)

def main():
    rclpy.init()
    node = rclpy.create_node('pointcloud_listener')
    subscriber = node.create_subscription(PointCloud2, 's17_vision/not_space/points', callback, 10)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
