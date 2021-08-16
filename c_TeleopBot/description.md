# Teleop Bot
- 다양한 요소 때문에 사람이 상세하게 조종하는 것이 표준 관행인 분야가 많이 있으며 **원격 제어** 에 대해 공부한다.
- 차동 구동 평면 로봇을 구동할 때에는 선속도, 각속도를 사용하여 필요한 로봇의 바퀴 속도를 계산한다.
  - 바퀴 간격, 직경의 함수로 계산하기 위해 삼각함수를 활용한다.
- 원격 제어 소프트웨어의 관점에서 선속도 : m/s, 각속도 : rad/s로 명령을 내린다.
  - 위 속도 명령 스트림을 만들어야 한다면, 그 다음으로는 어떻게 이 **명령을 유도해 낼 수 있는가**이다.
## 개발 패턴
- 여러 개의 작은 ROS 노드들을 생성하면 수동 및 자동 소프트웨어 테스트를 만들기 쉽다.
  - 키 입력 -> 운동명령 변환 Node => 올바른 응답과 비교. (자동화된 테스트도 준비 가능해진다.)
## KeyBoard 구동기
- key\_publisher.py
```python
import sys, select, tty, termios
import rospy
from std_msgs.msg import String

if __name__ = "__main__":
    key_pub = rospy.Publisher("keys", String, queue_size=1)
    rospy.init_node("keyboard_driver")
    rate = rospy.Rate(100)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print "Publishing keystrokes. Press Ctrl-C to exit..."
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
        rate.sleep()
    termios.tcsetattr(sys.stdin, temios.TCSADRAIN, old_attr)
```
- sys.stdin.fileno() 는 stdin 표준 입력 스트림의 파일 디스크립터에 대한 번호를 출력해준다.
  - [참고링크](https://stackoverflow.com/questions/32199552/what-is-sys-stdin-fileno-in-python)
```python
old_attr = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())
```
- termios : raw keystroke 을 받아 들이기 위해 사용하는 라이브러리
  - 보통 콘솔은 전체 텍스트 줄을 버퍼에 넣어서 엔터 키를 누를 때 프로그램으로 보내준다.
- 누르자마자 곧바로 프로그램의 표준 입력 스트림으로 키를 받기 원하므로 콘솔의 이런 행동을 변경하기 위해 먼저 속성을 저장한다.
- tty.setcbreak 은 파일 디스크립터의 모드를 cbreak로 변경한다. 기본 값은 termios.TCSAFLUSH이고 termiostcsetattr()로 전달된다.
  - [참고링크](https://docs.python.org/ko/3/library/tty.html)
  - TCSAFLUSH : 계류 중인 모든 출력을 전송하고 계류 중인 모든 입력을 버린 후 변경하려면 TCSAFLUSH. ([참고링크](https://docs.python.org/ko/3/library/termios.html#module-termios))
```python
if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
    key_pub.publish(sys.stdin.read(1))
rate.sleep()
```
- 글자가 입력되었는지 보기 위해 stdin 스트림을 계속 **Polling** 할 수 있다.
- sys.stdin.read(1) 은 blocking 함수 이므로 글자가 입력되기 전까지 반환 되지 않는다. 그러므로 여기서 프로그램 수행을 멈출 수 있다.
- stdin을 차단하면 나중에 무언가를 추가하려고 할 때 프로세스가 ROS 콜백을 시작하지 못하게 되므로 글자를 바로 반환되도록 timeout을 0 으로 하여 select()를 호출하는 것이 좋은 습관이다.
  - select() 는 시스템 호출 관련 인터페이스다. 0 이 timeout이다.
  - [참고링크](https://sites.google.com/site/justinohahm/home/rclab/paisseon-peulogeulaeming-yeonseub?tmpl=%2Fsystem%2Fapp%2Ftemplates%2Fprint%2F&showPrintDialog=1)
- rate.sleep() 에서 나머지 반복 시간을 보낼 것이다.
```python
termios.tcsetattr(sys.stdin, temios.TCSADRAIN, old_attr)
```
- 프로그램을 종료하기 전에 콘솔을 표준 모드로 되돌리는 코드다.
```shell
./keys_publisher.py
```
```shell
rostopic echo keys
```
![image](https://user-images.githubusercontent.com/69780812/128982224-1c584561-070f-429a-bb19-c9673e76e9eb.png)
- 다른 터미널 창에서 echo 하면 publisher 실행한 터미널에서의 키보드 값 출력을 얻을 수 있다.
  - 위의 코드에서는 일반적인 키는 예상한 대로 동작하지만 화살표와 같은 확장된 키는 이상하게 작동한다.
  - stdin은 한 번에 한 글자씩 가져오기 때문인데, 나중에 공부해보면 좋겠다.
##
- keys\_to\_twist.py
```python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = { 'w': [0, 1], 'x': [0, -1],
                'a': [-1, 0], 'd': [1, 0],
                's': [0, 0] }

def keys_cb(msg, twist_pub):
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return
    vels = key_mapping[msg.data[0]]
    t = Twist()
    t.angular.z = vels[0]
    t.linear.x = vels[1]
    twist_pub.publish(t)

if __name__ == "__main__":
    rospy.init_node("keys_to_twist")
    twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("keys", String, keys_cb, twist_pub)
    rospy.spin()
```
```python
key_mapping = { 'w': [0, 1], 'x': [0, -1],
                'a': [-1, 0], 'd': [1, 0],
                's': [0, 0] }
```
- 키 입력과 목표 속도 사이의 mapping을 저장한다.
- w,x,a,d,s 로 로봇을 제어하려 하는 것이다.
```python
if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
    return
```
- keys 토픽에 대한 콜백 함수는 키를 이 딕셔너리로 검색하는데, 키를 찾으면 목표 속도가 딕셔너리로 부터 추출된다.
  - 콜백 함수는 callback 인자로 twist\_pub을 받는 오버라이딩된 형태를 사용한다.
```shell
./keys_publisher.py
```
```shell
./keys_to_twist.py
```
```shell
rostopic echo cmd_vel
```
- publisher 창에서 키를 입력하면 echo 하고 있는 창에서 twist 메시지의 값들이 나온다.
##
- keys\_to\_twist\_usiing\_rate
```python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = { 'w': [0, 1], 'x': [0, -1],
                'a': [-1, 0], 'd': [1, 0],
                's': [0, 0] }

def keys_cb(msg, twist_pub):
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return
    vels = key_mapping[msg.data[0]]
    t = Twist()
    t.angular.z = vels[0]
    t.linear.x = vels[1]
    twist_pub.publish(t)

if __name__ == "__main__":
    rospy.init_node("keys_to_twist")
    twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("keys", String, keys_cb, twist_pub)
    rate = rospy.Rate(10)
    g_last_twist = Twist()
    while not rospy.is_shutdown():
        twist_pub.publish(g_last_twist)
        rate.sleep()
```
- 이전 프로그램의 핵심적인 차이는 rospy.Rate()의 사용이다.
- 지속적인 속도 명령 스트림을 요구하는 로봇에 유용하다.
- 일정한 결과를 얻도록 반복문 실행에 소비되는 시간을 지속적으로 추정하는 ROS rate구조로 실행한 것이다.
  - 보통 컴퓨터마다 계산 성능이 다르므로 반복문이 특정한 갱신 비율을 유지하게 하는 것이 어려워 CPU 시간을 미리 알기 어렵다.

![image](https://user-images.githubusercontent.com/69780812/128989873-567e62be-fd45-4b2f-9b6e-efd6bdf6bb20.png)
- cmd\_vel 의 토픽 자료형에 대해 알고 싶다면 ?
```shell
rostopic info cmd_vel
```
- geometry\_msgs/Twist형이라는 것을 알게 될 것이고
```shell
rosmsg show geometry_msgs/Twist
```
- linear, angular이 float64 자료형이라는 것도 알 수 있다.
```shell
rqt_plot cmd_vel/linear/x cmd_vel/angular/z
```
- plot으로 디버깅 하는 것도 좋은 방법 중 하나이다.
## 매개변수 서버
- roscore(ROS Master)는 모든 ROS노드와 명령행 도구가 읽고 쓸 수 있는 매개변수 서버를 포함하고 있다.
- 일반적인 키/값 저장소다.
- Private 매개변수 이름을 추가함으로써 형성된다. 그저 노드 이름에 매개변수 이름을 추가한다는 것을 의미한다.
  - 이는 이름 충돌이 일어날 수 없음을 보장한다.
  - 예를들어, keys\_to\_twist 라는 노드 이름이 있으면 keys\_to\_twist/linear\_scale 이라고 이름 지어진 Private매개변수를 가질 수 있다.
  - \_ 밑줄로 시작하는 매개변수 이름을 쓰고 := 형식을 사용하여 값을 설정한다.
```shell
./keys_to_twsit_parameterized.py _linear_scale:=0.5 _angular_scale:=0.4
```
- 이것은 노드가 시작되기 전에 매개변수를 각각 설정하는 예이다.
- 이들 매개변수 값은 has\_param()과 get\_param() 호출로 반환된다. 
##
- keys\_to\_twist\_parameterized.py
```python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = { 'w': [0, 1], 'x': [0, -1],
                'a': [-1, 0], 'd': [1, 0],
                's': [0, 0] }

g_last_twist = None
g_vel_scales = [0.1, 0.1]

def keys_cb(msg, twist_pub):
    global g_last_twist, g_vel_scales
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return
    vels = key_mapping[msg.data[0]]
    g_last_twist.angular.z = vels[0] * g_vel_scales[0]
    g_last_twist.linear.x = vels[1] * g_vel_scales[1]
    twist_pub.publish(g_last_twist)

if __name__ == "__main__":
    rospy.init_node("keys_to_twist")
    twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("keys", String, keys_cb, twist_pub)
    rate = rospy.Rate(10)
    g_last_twist = Twist()
    if rospy.has_param("~linear_scale"):
        g_vel_scales[1] = rospy.get_param("~linear_scale")
    else:
        rospy.logwarn("linear scale not provided; using % .1f" % g_vel_scales[1])

    if rospy.has_param("~angular_scale"):
        g_vel_scales[0] = rospy.get_param("~angular_scale")
    else:
        rospy.logwarn("angular scale not provided; using % .1f" % g_vel_scales[0])

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        twist_pub.publish(g_last_twist)
        rate.sleep()
```
- 시작할 때 이 프로그램은 has\_param, get\_param 을 사용하여 매개 변수 서버에 Query 하고 지정된 매개변수가 설정되어 있지 않으면 경고를 출력한다.
- get\_param() 함수는 선택적으로 두 번째 매개변수를 받는데, 매개변수의 키가 매개변수 서버에 존재하지 않으면 기본 매개변수로 처리된다. 
##
- keys\_to\_twist\_with\_ramps.py
```python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = {'w': [0, 1], 'x': [0, -1], 'a': [1, 0], 'd': [-1, 0], 's': [0, 0]}

g_twist_pub = None
g_target_twist = None
g_last_twist = None
g_last_send_time = None
g_vel_scales = [0.1, 0.1]
g_vel_rapms = [1, 1]

def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
    step = ramp_rate * (t_now - t_prev).to_sec()
    sign = 1.0 if (v_target > v_prev) else -1.0
    error = math.fabs(v_target - v_prev)
    if error < step:
        return v_target
    else:
        return v_prev + sign * step

def ramped_twist(prev, target, t_prev, t_now, ramps):
    tw = Twist()
    tw.angular.z = ramped_vel(prev.augular.z, target.anuglar.z, t_prev, t_now, ramps[0])
    tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev, t_now, rapms[1])

    return tw

def send_twist():
    global g_last_twist_send_time, g_target_twist, g_last_twist,\
           g_vel_scales, g_vel_ramps, g_twist_pub

    t_now = rospy.Time.now()
    g_last_twist = ramped_twist(g_last_twist, g_target_twist, g_last_twist_send_time, t_now, g_vel_rapms)
    g_last_twist_send_time = t_now
    g_twist_pub.publish(g_last_twist)

def key_cb(msg):
    global g_target_twist, g_last_twist, g_vel_scales
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return
    vels = key_mapping[msg.data[0]]
    g_target_twist.angular.z = vels[0] * g_vel_scales[0]
    g_target_twist.linear.x = vels[1] * g_vel_scales[1]

def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print "parameter [%s] not defined. Defaulting to %.3f" % (name, default)

    return default

if __name__ == "__main__":
    rospy.init_node("key_to_twist")
    g_last_twist_send_time = rospy.Time.now()
    g_twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("keys", String, keys_cb)
    g_target_twist = Twist()
    g_last_twist = Twist()
    g_vel_scales[0] = fetch_param("~angular_scale", 0.1)
    g_vel_scales[1] = fetch_param("~linear_scale", 0.1)
    g_vel_ramps[0] = fetch_param("~angular_accel", 1.0)
    g_vel_ramps[1] = fetch_param("~linear_scale", 1.0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        send_twist()
        rate.sleep()
```
```python
def ramped_twist(prev, target, t_prev, t_now, ramps):
    tw = Twist()
    tw.angular.z = ramped_vel(prev.augular.z, target.anuglar.z, t_prev, t_now, ramps[0])
    tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev, t_now, rapms[1])

    return tw
```
- 관심있는 주요 부분이 위 함수이다.
- 속도는 매개변수로 제공된 가속 상수 아래에서 계산된다.
- 목표 속도를 향해 한 단계 더 앞서거나 혹은 목표 속도가 한 단계 내에 떨어져 있으면 그 값으로 간다.

![image](https://user-images.githubusercontent.com/69780812/129157237-3e0667b4-5302-40fb-8928-abe72f92a9b2.png)

- 명령이 올라가거나 내려가는 데 시간이 다소 걸린다. 터틀봇에 순간적으로 변하는 step 명령을 주더라도 신호 경로의 어딘가에서 혹은 기계적인 시스템으로 인해 물리적으로 계단 명령은 경사 형태로 바뀔 것이다.(?)
```shell
./keys_to_twist_with_ramps.py
```
```shell
./keys_publisher.py
```
```shell
roslaunch turtlebot3_gazebo turtilebot3_house.launch
```

![image](https://user-images.githubusercontent.com/69780812/129161095-34f18ec5-eee0-4585-9315-dea7de209335.png)

## Rviz
- ROS 시각화를 나타낸다.
- roscore, 키보드 구동기, 제어값 변환 노드, 시뮬레이션, rivz 이렇게 5개의 터미널이 필요하다.
```shell
rosrun rviz rviz
```
- ROS에서 모든 형태의 데이터는 **참조 프레임에 부착**된다.
  - 카메라는 터틀봇 이동체의 중심에 대해 상대적으로 정의된 참조 프레임에 부착
  - odom(odometry 참조프레임)은 관례상 로봇에 전원이 공급되거나 odometry가 가장 최근에 리셋된 위치에서 원점을 가지도록 한다. 종종 어깨위 추적 시점을 가진다.
- rviz는 낮우에 재사용할 수 있도록 시각화 상태를 설정 파일로 저장할 수 있다. 다음번 rviz를 실행할 때 불러올 수 있으며 패널과 플러그인을 똑같이 설정할 수 있다.
- 
