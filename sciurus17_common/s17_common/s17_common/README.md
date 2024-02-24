# s17_common API
## RobotControl
　簡易的にロボットの各関節を制御します
```python
from s17_common.robot_control import RobotControl

# ROS2 rclpy
from rclpy.node import Node
import rclpy

rclpy.init()
node = Node('sample_code')

# node オブジェクトを代入してインスタンス化
rc = RobotControl(node)
```

### left_arm
　左手の関節を制御します。制御したいジョイント
```joint1 ~ joint7```
を引数に弧度法の角度値を入力してください。

### right_arm
　右手の関節を制御します。制御したいジョイント
```joint1 ~ joint7```
を引数に弧度法の角度値を入力してください。

### neck
　首関節を制御します。```pitch, yaw``` 任意引数をとり、弧度法で角度値を入力してください。

### waist_yaw
　胴体の yaw 軸を制御します。必須引数 ```yaw``` に弧度法で角度値を入力してください。

### init_pose
　初期姿勢（カスタム）に遷移します。

### t_pose
　Tの字の姿勢を取ります。
