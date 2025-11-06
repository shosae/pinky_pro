# pinky_pro
ROS2 packages for Pinky Pro

## 🙏 Special Thanks · Contributors

**[byeongkyu](https://github.com/byeongkyu)** – Pinky PRO 모델 ROS 2 패키지 개발  
참고 레포지토리: [pinky_robot](https://github.com/byeongkyu/pinky_robot)

# PC 설정
## 환경
- ubuntu 24.04
- ros2 jazzy
- x86_64 (amd64) (recommended)
    - (ARM64 환경의 경우 [관련 문서](doc/arm64_guide.md)를 참고하세요.)
## 1. Pinky Pro ROS2 pkg clone
```
mkdir -p ~/pinky_pro/src
cd ~/pinky_pro/src
git clone https://github.com/pinklab-art/pinky_pro.git
```
## 2. dependency 설치
```
cd ~/pinky_pro
rosdep install --from-paths src --ignore-src -r -y
```
## 3. build
```
cd ~/pinky_pro
colcon build
```

# Pinky Pro 사용 매뉴얼

## 환경
- ubuntu 24.04
- ros2 jazzy

## pinky Pro 실행
```
ros2 launch pinky_bringup bringup_robot.launch.xml
```

## Map building
#### launch slam toolbox
```
ros2 launch pinky_navigation map_building.launch.xml
```
#### [ONLY PC] map view 
```
ros2 launch pinky_navigation map_view.launch.xml
```
#### robot keyborad control
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
#### map save 
```
ros2 run nav2_map_server map_saver_cli -f <map name>
```

## Navigation2 
#### launch navigation2
```
ros2 launch pinky_navigation bringup_launch.xml map:=<map name>
```

#### [ONLY PC] nav2 view
```
ros2 launch pinky_navigation nav2_view.launch.xml
```

# 시뮬레이션
## pinky Pro gazebo 실행
#### 가제보 실행
```
ros2 launch pinky_gz_sim launch_sim.launch.xml
```

## Map building
#### launch slam toolbox
```
ros2 launch pinky_navigation map_building.launch.xml use_sim_time:=true
```
#### [ONLY PC] map view 
```
ros2 launch pinky_navigation map_view.launch.xml
```
#### robot keyborad control
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
#### map save 
```
ros2 run nav2_map_server map_saver_cli -f <map name>
```

## Navigation2 
#### launch navigation2
```
ros2 launch pinky_navigation bringup_launch.xml map:=<map name> use_sim_time:=true
```

#### [ONLY PC] nav2 view
```
ros2 launch pinky_navigation nav2_view.launch.xml
```

# 센서 동작
## LED contorl
### LED server start
```
ros2 launch pinky_navigation bringup_launch.xml map:=<map name> use_sim_time:=true
```
### LED service call
#### fill with color
```
ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'fill', r: 255, g: 0, b: 0}"
```
#### set pixel colors
```
ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'set_pixel', pixels: [4, 5, 6, 7], r: 0, g: 0, b: 255}"
```
#### clear
```
ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'clear'}"
```
#### set brightness
```
ros2 service call /set_brightness pinky_interfaces/srv/SetBrightness "{brightness: 10}"
```
## LCD contorl
### emotion server start
```
ros2 run pinky_emotion emotion_server
```
or
```
ros2 run pinky_emotion emotion_server --ros-args -p load_frame_skip:=3
```

### set emotion
Available emotions: (hello, basic, angry, bored, fun, happy, interest, sad)
```
ros2 service call /set_emotion pinky_interfaces/srv/Emotion "{emotion: 'happy'}"
```
