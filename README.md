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

- ロボットがいない状態<br>
    <img src="https://gitlab.com/nakatogawalabolatory/docker/turtlebot_sim_docker/-/blob/main/img/Screenshot%20from%202023-10-26%2007-24-12.png" width="300">
- ロボットがいる状態（正常）<br>
    <img src="https://gitlab.com/nakatogawalabolatory/docker/turtlebot_sim_docker/-/raw/main/img/Screenshot%20from%202023-10-26%2007-24-49.png" width="300">
