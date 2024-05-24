# ai_car 操作手冊

## Raspberry Pi
確定車子能正常的連上網路後下列步驟才能正確安裝

1. 建立 ROS 工作區（若已建立可以直接下一步）

```
mkdir -p catkin_ws/src
```

2. 下載 ai_car 功能包（請留意 catkin_ws/src 中不要有 ai_car 資料夾才能正常下載）

```
cd ~/caktin_ws/src && git clone https://github.com/zhl017/ai_car.git
```

3. 編譯 ai_car 功能包

```
cd ~/catkin_ws && catkin_make
```

4. 載入 catkin_ws 環境設定

```
source ~/catkin_ws/devel/setup.bash
```

5. 安裝套件 rplidar_ros, RTIMU

```
cd ~/catkin_ws/src/ai_car && bash install_ai_car
```

6. 待 PC 端 `ROS MASTER` 運行後使用 `roslaunch` 運行車子

```
roslaunch ai_car ai_car.launch
```

## PC

1. 建立 ROS 工作區（若已建立可以直接下一步）

```
mkdir -p catkin_ws/src
```

2. 下載 ai_car 功能包（請留意 catkin_ws/src 中不要有 ai_car 資料夾才能正常下載）

```
cd ~/caktin_ws/src && git clone https://github.com/zhl017/ai_car.git
```

3. 編譯 ai_car 功能包

```
cd ~/catkin_ws && catkin_make
```

4. 載入 catkin_ws 環境設定

```
source ~/catkin_ws/devel/setup.bash
```

5. 運行 `ROS Master`

```
roscore
```

### 基本功能指令

* 遙控程式

```
rosrun ai_car aicar_teleop.py
```

* 觀察鏡頭畫面

```
rqt_image_view
```

* rviz 看車子

可以在 rviz 中觀看車子的光達資訊以及移動過程

```
roslaunch ai_car display.launch
```

### SLAM

> 請確定 PC 正常運行 ROS MASTER 及 車子正在運行 ai_car.launch

> 若運行過程中出現錯誤顯示未找到套件,請使用指令 sudo apt install ros-noetic-套件名稱 進行安裝
> ex. 
> ERROR: cannot launch node of type [gmapping/slam_gmapping]
> 表示沒有 slam_gmapping 套件,使用指令安裝
> sudo apt install ros-noetic-slam-gmapping

1. 運行 `slam.launch`

```
roslaunch ai_car slam.launch
```

2. 使用遙控程式控制車子移動建立地圖

```
rosrun ai_car aicar_teleop.py
```

3. 建立完成後地圖使用 `map_server` 儲存

```
rosrun map_server map_saver -f ~/map
```

4. 關閉 `slam.launch` 與 遙控程式

### 導航

> 請確定 PC 正常運行 ROS MASTER 及 車子正在運行 ai_car.launch

> 若運行過程中出現錯誤顯示未找到套件,請使用指令 sudo apt install ros-noetic-套件名稱 進行安裝
> ex. 
> ERROR: cannot launch node of type [gmapping/slam_gmapping]
> 表示沒有 slam_gmapping 套件,使用指令安裝
> sudo apt install ros-noetic-slam-gmapping

1. 運行 `nav.launch`

```
roslaunch ai_car nav.launch map_file:=$HOME/map.yaml
```

2. 點選 rviz 上方工具欄**第一個**綠色箭頭 **2D Pose Estimate**, 在地圖中賦予車子初始位置, 賦予位置時滑鼠左鍵按住可以進行旋轉, 箭頭方向為車子面朝方向, 即面朝 **X** 的方向

3. 點擊 rviz 上方工具欄**第二個**綠色箭頭 **2d Nav Goal**, 在地圖中賦予車子移動的終點目標, 賦予位置時滑鼠左鍵按住可以進行旋轉, 箭頭方向為車子面朝方向, 即面朝 **X** 的方向
