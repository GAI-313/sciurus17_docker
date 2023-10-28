# turtlebot3 simulation in docker
　エントリー
```bash
docker-compose up -d --build turtlebot3; docker-compose exec turtlebot3 /bin/bash
```
　コンテナ内で Gazebo シミュレーションを立ち上げるには、以下のコマンドを１度実行。
```bash
ros2 launch tb3_common turtlebot3_sim_minimum_bringup.launch.py rviz_view:=true
```
　しばらく待つと Gazebo シミュレーションが立ち上がる。このとき下の図のようにロボットがいなければもう一度この起動コマンドを実行する。

- ロボットがいない状態<br>
    <img src="https://gitlab.com/nakatogawalabolatory/docker/turtlebot_sim_docker/-/raw/main/img/Screenshot%20from%202023-10-26%2007-24-12.png" width="300">
- ロボットがいる状態（正常）<br>
    <img src="https://gitlab.com/nakatogawalabolatory/docker/turtlebot_sim_docker/-/raw/main/img/Screenshot%20from%202023-10-26%2007-24-49.png" width="300">

## ロボットをコントロールする
  Nintendo Switch Pro Controller で Turtlebot3 を制御する場合は、デフォルトでジョイスティックによる制御が有効になっている。<br>
　keyboard teleop を実行するときは、新たなターミナルで
```bash
docker-compose exec turtlebot3 /bin/bash
```
を実行し、コンテナ内で
```
ros2 run turtlebot3_teleop teleop_keyboard
```
を実行する。

## Rviz を非表示にする。
　```turtlebot3_sim_minimum_bringup.launch.py```
が起動するとデフォルトで RViz が起動する。これを防ぎたい場合は、
```bash
ros2 launch tb3_common turtlebot3_sim_minimum_bringup.launch.py rviz_view:=false
```
　のように、実行引数
```rviz_view```
を
```false```
にすればよい。
## ローカルのパッケージをコンテナ内に持ってくる
　ローカルの ROS2 パッケージを使ってコンテナ内のTurtlebot3を制御したい場合は、このリポジトリディレクトリ内にある
```.env``` ファイルを編集する。
```bash
nano .env
# or
vim .emv
```
　```.env``` ファイル内はこのようになっている。
```
USER_NAME=nakalab
GROUP_NAME=nakalab
UID=1000
GID=1000

# 絶対パスで記述してください
#PKGS_PATH_ABS = /path/to/colcon_ws/src
```
　この
```
# 絶対パスで記述してください
PKGS_PATH_ABS = /path/to/colcon_ws/src
```
の
```
PKGS_PATH_ABS = /path/to/colcon_ws/src
```
に記述されているパスを
**絶対パス**
でワークスペースディレクトリの
```/src```
までのパスに書き換えてください。<br>
　次に、
```docker-compose.yaml```
の25行目のコメントを外す。
```
18	    volumes:
19	       - /tmp/.X11-unix:/tmp/.X11-unix
20	       #- /tmp/pulseaudio.socket:/tmp/pulseaudio.socket
21	       #- /tmp/pulseaudio.client.conf:/etc/pulse/client.conf
22	       - $HOME/.Xauthority/:/root/.Xauthority
23	       - /dev:/dev
24	       - ./turtlebot3_common:/colcon_ws/src/turtlebot3_common
25	       #- $PKGS_PATH_ABS:/colcon_ws/src/extra_pkgs <- ここのコメントを外す
26	    devices:
27	       - /dev:/dev
```
　そしたらコンテナにエントリーする。
```bash
docker-compose up -d --build turtlebot3; docker-compose exec turtlebot3 /bin/bash
```
　コンテナ内に入ったら以下のコマンドを実行する。
```
rosdep install -y -i --from-path src; colcon build --symlink-install; source ~/.bashrc
```
