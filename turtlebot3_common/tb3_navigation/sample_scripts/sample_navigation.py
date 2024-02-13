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
from geometry_msgs.msg import PoseWithCovarianceStamped # 位置を変更するメッセージ
from geometry_msgs.msg import Twist

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

    def get_current_pose(self, xyy=False):
        while rclpy.ok():
            rclpy.spin_once(self.node)
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                if xyy:
                    p = []
                    p.append(transform.transform.translation.x)
                    p.append(transform.transform.translation.y)

                    e = euler_from_quaternion((transform.transform.rotation.x,
                                                transform.transform.rotation.y,
                                                transform.transform.rotation.z,
                                                transform.transform.rotation.w))
                    p.append(e[2])

                    transform = p
                    
                return transform
            except Exception as e:
                #print(e)
                pass

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

    # 初期位置を更新するメソッド
    def pose_estimate(self, x=0.0, y=0.0, yaw=0.0, adjust=False):
        pub = self.node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        p = PoseWithCovarianceStamped()
        #quaternion に変換
        q = quaternion_from_euler(0.0,0.0,yaw)
        # 目標座標を設定
        p.header.frame_id = 'map'
        p.header.stamp = self.navigator.get_clock().now().to_msg()
        p.pose.pose.position.x = float(x)
        p.pose.pose.position.y = float(y)
        p.pose.pose.orientation.x = q[0]
        p.pose.pose.orientation.y = q[1]
        p.pose.pose.orientation.z = q[2]
        p.pose.pose.orientation.w = q[3]

        pub.publish(p)

        if adjust:
            self.go_rlt(yaw=1.0)
            self.go_rlt(yaw=-2.0)
            self.go_rlt(yaw=1.0)
            self.go_abs(x, y, yaw)

    # ウェイポイントを作成するメソッド
    def create_waypoint(self, interval_distance=0.1, interval_angle=0.7):
        init_time = time.time()
        wp = []
        n = 0
        while True:
            try:
                # ウェイポイントに追加
                p = self.get_current_pose(xyy=True)
                if len(wp) == 0:
                    print('Add start waypoint!')
                    wp.append(p)
                    n += 1
                else:
                    if abs(wp[n-1][0] - p[0]) >= interval_distance or abs(wp[n-1][1] - p[1]) >= interval_distance or abs(wp[n-1][2] - p[2]) >= interval_angle:
                        print('Add waypoint!')
                        wp.append(p)
                        n += 1

            except KeyboardInterrupt:
                self.logger.info('waypoint creator break')
                break
        return wp

    def go_waypoint(self, waypoint):
        for w in waypoint:
            self.go_abs(w[0], w[1], w[2])

# メイン関数
def main():
    # rclpy を初期化してノードを宣言
    rclpy.init()
    node = Node('sample_navigation')

    # ノードをクラスにわたしてインスタンス化。（本家推奨の書き方ではないが柔軟に対応できる記述法）
    sn = SampleNavigation(node)

    # マップ基準で 1m 前に進む
    sn.go_abs(1.0)

    # 現在位置を基準に 2m 前に進む
    sn.go_rlt(2.0)

    # マップの 0 座標へ移動する
    sn.go_abs()
    '''
    # ウェイポイントを作成 (0.5m おきにウェイポイントを作成)
    waypoint = sn.create_waypoint(interval_distance=0.5)
    print(waypoint)
    '''
    # ウェイポイントを実行
    waypoint = [[4.841448938730508e-05, 1.7914445166189452e-07, 0.0011305815562633061], [-0.0011628905086995545, -0.0005248392866043505, 0.7072508218733972], [-0.045769045650673856, -0.002958355394447526, -0.0033677755747006047], [-0.02012383158848996, 0.053448537765798276, 0.7230624221692022], [0.15508943230660854, 0.5597348027882587, 1.19714110319368], [0.2566328200438195, 0.9333926174576131, 0.48279340649954305], [0.7590107956334997, 1.0176098074250812, -0.06771391743071824], [1.259669010757435, 0.9729782337101347, -0.11214139545441439], [1.4858207080665944, 0.943824112500417, -0.8425976513798746], [1.5709546357433666, 0.4403228958497772, -1.5132549713582124], [1.6063337390284858, -0.06054100206947849, -1.518348134441963], [1.4961152307285968, -0.5658424936342826, -1.7573067761817007], [1.405089652381129, -1.0675014011787556, -1.771563173349732], [1.4021294305967045, -1.1317059439692432, -2.489491211164555], [1.3876610483842915, -1.1173625613547802, 3.1118384468089566], [0.881633100598697, -1.0552430249486453, 3.0000252041724256], [0.5258625121204996, -1.0011156391626255, 2.2897884275870215]]
    sn.go_waypoint(waypoint)

# 単体で実行されたらコールされる
if __name__ == '__main__':
    main()
