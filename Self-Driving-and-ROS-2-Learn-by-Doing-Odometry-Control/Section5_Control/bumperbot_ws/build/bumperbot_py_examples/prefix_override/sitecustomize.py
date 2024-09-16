import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dev/ros2_ws/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control/Section5_Control/bumperbot_ws/install/bumperbot_py_examples'
