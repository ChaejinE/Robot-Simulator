#WanderBot
```shell
mkdir -p ./wanderbot_ws/src
cd ~/wanderbot_ws/src
catkin_init_workspace
```
```shell
cd ~/wanderbot_ws/src
catkin_create_pkg waderbot rospy geometry_msgs sensor_msgs
```
- CMakeLists.txt : 이 패키지에 대한 빌드 스크립트 시작점 이다.
- package.xml : 이름, 설명, 저자, 라이선스와 빌드 및 실행을 위해 의존해야 하는 다른 패키지에 관한 상세한 내용을 포함하는 기계 판독 가능한 패키지 설명
```shell
sudo apt-get install ros-melodic-turtlebot3-gazebo
```
- turtlebot이 버전이 바뀔 수 있기 때문에 확인이 필요한 부분이다.
```shell
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```
- 나는 waffle\_pi 모델을 사용했고, burger 모델도 있다.
```shell
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
- 잘 되는지 실행해보자.
```python
import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
rospy.init_node("red_light_green_light")

red_light_twist = Twist()
green_light_twist = Twist()
green_light_twist.linear.x = 0.5

driving_forward = False
light_change_time = rospy.Time.now()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if driving_forward:
        cmd_vel_pub.publish(green_light_twsit)
    else:
        cmd_vel_pub.publish(red_light_twist)

    if light_change_time < rospy.Time.now():
        driving_forward = not driving_forward
        light_change_time = rospy.Time.now() + rospy.Duration(3)

    rate.sleep()
```
- red\_light\_green\_light.py 예제다.
```shell
chmod +x red_light_green_light.py
./red_light_green_light.py cmd_vel:=cmd_vel_mux/input/teleop
```
- 예제를 실행해본다.
- 터틀봇 소프크웨어 스택이 기대하는 토픽으로 Twist 메시지를 발행할 수 있도록 cmd\_vel 토픽 이름을 변경한다.
  - 다른 로봇 소프크웨어 스택에서 어떤 토픽 이름을 요구하더라도 cmd\_vel 토픽 이름을 쉽게 변경할 수 있다.
![image](https://user-images.githubusercontent.com/69780812/128675099-57e4bc28-40fa-4aaf-910e-0aa0338edda5.png)
- 터틀봇이 3초마다 주행과 정지를 교대로 반복한다.
```python
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
```
- queue\_size : rospy 에게 발신 메시지를 몇개 버퍼링 할지 정한다.
- 메시지를 Publish 하는 노드가 Subscribe 하는 노드가 받을 수 있는 속도보다 빠르게 보내고 있다면 rospy는 queue\_size 개수 이상의 메시지는 단순히 폐기한다.
```python
red_light_twist = Twist()
green_light_twist = Twist()
green_light_twist.linear.x = 0.5
```
- red는 정지, green은 주행을 뜻한다.
- Twist 메시지의 선형 속도는 관례적으로 로봇이 향하고 있는 방향으로 정렬된다. 따라서 이는 0.5m/s로 직진하라는 뜻이다.
```shell
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
```shell
rostopic echo scan
```
- LaserScan 메시지의 ranges멤버를 주목한다.

```python
brearing = msg.angle_min + i * msg.angle_max / len(msg.ranges)
```
- 메시지 인스턴스가 msg로 명명되어 있다고 가정하면 특정 거리 추정 값을 위한 방향은 위와 같이 계산할 수 있다.
- 로봇의 바로 정면에 있는 가장 가까운 장애물 까지의 거리를 추출하려면 ranges 배열의 중간 원소를 선택하면 된다.
```python
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    range_ahead = msg.ranges[len(msg.ranges)/2]
    print "range ahead : %0.1f" % range_ahead
    
rospy.init_node("range_ahead")

scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)
rospy.spin()
```
- range\_ahead.py 파일을 만들어 보았다.
- 이 작은 프로그램은 ROS에서 데이터 스트림을 연결하고 파이썬에서 처리하는 것이 얼마나 쉬운지를 보여준다.
- LaserScan 메시지의 ranges 필드 중간 원소를 선택함으로써 로봇 바로 정면에 있는 객체까지 측정된 거리를 출력한다. 
  - ranges는 float\[]32 자료형이다.
![image](https://user-images.githubusercontent.com/69780812/128681456-ebe671a5-cc57-4091-99cc-9ac5296f71d0.png)
## WanderBot Sensing + Actuation
```python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)

g_range_ahead = 1
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
rospy.init_node("wander")
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if driving_forward:
        if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
            driving_forward = False
            state_change_time = rospy.Time.now() + rospy.Duration(5)

    else:
        if rospy.Time.now() > state_change_time:
            driving_forward  = True
            state_change_time = rospy.Time.now() + rospy.Duration(30)

    twist = Twist()
    if driving_forward:
        twist.linear.x = 1
    else:
        twist.angular.z = 1

    cmd_vel_pub.publish(twist)

    rate.sleep()
```
- 터틀봇이 0.8m 내의 장애물을 보거나 30초 후에 타임아웃 될때까지 직진 주행한 다음 멈추고 새로운 방향으로 제자리에서 맴돌도록 하는 예제
```python
if driving_forward:
        if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
            driving_forward = False
            state_change_time = rospy.Time.now() + rospy.Duration(5)
```
- 30초 타임아웃이 경과하기 전까지 주행하거나 0.8m 이내의 장애물을 보면 not driving\_forward 상태가 된다.
```python
else:
        if rospy.Time.now() > state_change_time:
            driving_forward  = True
            state_change_time = rospy.Time.now() + rospy.Duration(30)
```
- not driving\_forward 상태에서는 5초 동안 제자리에서 단순히 회전만 할 것이다. 그런 다음 driving\_forward 상태로 돌아간다.
```shell
chmod +x wander.py
./wander.py cmd_vel:=cmd_vel_mux/input/teleop
```
- 장애물과의 충돌을 회피하며 끝없이 주변을 돌아 다닌다.
- 책에서는 cmd\_vel 을 다른 이름으로 remapping 하는데 turtlebot3 부터는 topic list를 확인해보니 cmd\_vel로 수정되어있었다. 그냥 remapping 안하는게 맞는 것 같다.
