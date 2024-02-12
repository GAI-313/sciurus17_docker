# ROS1 でいう rospy をインポートする
from rclpy.node import Node # ノード宣言時に使用する
import rclpy # ROS2 版 rospy

# Navigation2 API
from nav2_simple_commander.robot_navigator import BasicNavigator

# メッセージライブラリ
from geometry_msgs.msg import PoseStamped # 位置を定義するメッセージ

# 標準ライブラリ
import traceback # エラーをトレースバックする

# メインクラス
class SampleNavigation:
    # インスタンス化されたときに実行されるメソッド
    def __init__(self, node):
        self.node = node # ノード
        self.logger = self.node.get_logger()
        
        self.navigator = BasicNavigator()

        self.navigator.waitUntilNav2Active()
        self.logger.info('OK')

# メイン関数
def main():
    # rclpy を初期化してノードを宣言
    rclpy.init()
    node = Node('sample_navigation')

    # ノードをクラスにわたしてインスタンス化。（本家推奨の書き方ではないが柔軟に対応できる記述法）
    sn = SampleNavigation(node)

# 単体で実行されたらコールされる
if __name__ == '__main__':
    main()
