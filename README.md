# turtlebot3 simulation in docker
　エントリー
```bash
docker-compose up -d --build turtlebot3; docker-compose exec turtlebot3 /bin/bash
```
　コンテナ内で Gazebo シミュレーションを立ち上げるには、以下のコマンドを１度実行。
```bash
ros2 launch tb3_common turtlebot3_sim_minimum_bringup.launch.py
```
　しばらく待つと Gazebo シミュレーションが立ち上がる。このとき下の図のようにロボットがいなければもう一度この起動コマンドを実行する。

- ロボットがいない状態
    <img src="" width="300">
- ロボットがいる状態（正常）
    <img src="" width="300">
