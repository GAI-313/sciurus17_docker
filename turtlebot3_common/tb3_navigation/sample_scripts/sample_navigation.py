# ROS1 でいう rospy をインポートする
from rclpy.node import Node # ノード宣言時に使用する
import rclpy # ROS2 版 rospy

# tf2
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformListener, Buffer, TransformException

# Navigation2 API
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult

# メッセージライブラリ
from geometry_msgs.msg import PoseStamped # 位置を定義するメッセージ
from geometry_msgs.msg import PoseWithCovarianceStamped

# 標準ライブラリ
import traceback # エラーをトレースバックする
import threading
import time

# メインクラス
class SampleNavigation:
    # インスタンス化されたときに実行されるメソッド
    def __init__(self, node):
        self.node = node # ノード
        self.logger = self.node.get_logger() # logging 用
        self.navigator = BasicNavigator()

        # TF 関連
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        # Navigator が起動するのを待機
        self.navigator.waitUntilNav2Active()
        self.logger.info('OK')

    def get_current_pose(self):
        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                return transform
            except Exception as e:
                pass

            rclpy.spin_once(self.node)

    # 絶対座標で任意の座標へ向かうメソッド
    def go_abs(self, x=0, y=0, yaw=0):
        #quaternion に変換
        q = quaternion_from_euler(0.0,0.0,yaw)
        # 目標座標を設定
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.navigator.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]

        self.navigator.goToPose(p)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            #print(feedback)
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.logger.info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.logger.warn('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.logger.error('Goal failed!')
        else:
            self.logger.error('Goal has an invalid return status!')

    # 相対座標で任意の座標へ向かうメソッド
    def go_rlt(self, x=0, y=0, yaw=0):
        p = self.get_current_pose()

        px = p.transform.translation.x
        py = p.transform.translation.y

        e = euler_from_quaternion((p.transform.rotation.x,
                                    p.transform.rotation.y,
                                    p.transform.rotation.z,
                                    p.transform.rotation.w))
        pyaw = e[2]

        self.go_abs(px + x, py + y, pyaw + yaw)
        
# メイン関数
def main():
    # rclpy を初期化してノードを宣言
    rclpy.init()
    node = Node('sample_navigation')

    # ノードをクラスにわたしてインスタンス化。（本家推奨の書き方ではないが柔軟に対応できる記述法）
    sn = SampleNavigation(node)
    sn.go_rlt(yaw=1.57)

# 単体で実行されたらコールされる
if __name__ == '__main__':
    main()
