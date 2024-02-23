# Sciurus17_docker
　このパッケージはプライベートです。SSH key を事前に作成して使用することをおすすめします。
## インストールとセットアップ
1. **クローン**<br>
    ```bash
    git clone git@github.com:GAI-313/sciurus17_docker.git
    ```

2. **コンテナにエントリー**<br>
    ```bash
    ./entry_exec.bash
    ```
    　ビルドが成功すると自動的にコンテナにエントリーします。

## 起動
```bash
ros2 launch s17_bringup bringup.launch.py
```

## init_pose サービス
　別のターミナルでコンテナに入り、コンテナ内で以下のコマンドを実行するとロボットが私が定義した初期姿勢に遷移します。
```bash
ros2 service call /s17_common/init_pose std_srvs/srv/SetBool "{data: true}"
```

## 終了
　コンテナから出て、ホスト内で以下のコマンドを実行
```bash
docker compose down
```

# To Do
## SpaceFinder
　作業領域内の平面部を検出するノードを作成する。

## ObstacleDetection（作成中）
　コリジョンを検出し、Moveit の動作に対応させる。
