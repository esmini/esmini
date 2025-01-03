from pynput import keyboard
import threading
import signal
import rospy
import time
from math import pi
from std_msgs.msg import Float64
ACC_MAX = 10

current_pressed = set()
global current_acc
global current_steer
current_acc = 0
current_steer = 0

shutdown_event = threading.Event()

def signal_handler(sig, frame):
  exit(0)

def on_press(key):
  global current_acc
  global current_steer
  current_pressed.add(key)

  # Accel
  if keyboard.Key.up in current_pressed:
    if current_acc < ACC_MAX:
      current_acc += 0.2

  # Brake
  if keyboard.Key.down in current_pressed:
    if abs(current_acc) < ACC_MAX:
      current_acc -= 0.2

  if keyboard.Key.left in current_pressed:
    current_steer = 0.3

  if keyboard.Key.right in current_pressed:
    current_steer = -0.3

def on_release(key):
  global current_acc
  global current_steer
  if key == keyboard.Key.up or key == keyboard.Key.down:
    current_acc = 0
  elif key == keyboard.Key.right or key == keyboard.Key.left:
    current_steer = 0
  if key in current_pressed:
    current_pressed.remove(key)

if __name__ == '__main__':
  # Wait for ros master
  roscore_running = False
  while not roscore_running:
    try:
      rospy.get_master().getPid()
      roscore_running = True
    except Exception as e:
      print('Wait for ROS master for 1s')
      time.sleep(1)

  signal.signal(signal.SIGINT, signal_handler)
  listener_thread = keyboard.Listener(on_press=on_press, on_release=on_release)
  listener_thread.start()

  rospy.init_node('keyboard_controller')

  print('-' * 20)
  print('Keyboard Controller\nUp: acceleration\nDown: deceleration\nLeft, Right: steering')
  print('Ctrl+C: Terminate')
  print('-' * 20)

  acc_pub = rospy.Publisher('/esmini/accel', Float64, queue_size=10)
  steering_pub = rospy.Publisher('/esmini/steering', Float64, queue_size=10)

  rate = rospy.Rate(10)

  acc_msg = Float64()
  steering_msg = Float64()
  start_driving = False

  while not rospy.is_shutdown():
    acc_msg.data = current_acc
    steering_msg.data = current_steer

    acc_pub.publish(acc_msg)
    steering_pub.publish(steering_msg)

    if not start_driving and (current_acc != 0 or current_steer != 0):
      start_driving = True

    if start_driving:
      print('acc : %.2f mpss | steer : %.2f deg' % (current_acc, current_steer / pi * 180))

    rate.sleep()

  listener_thread.join()
