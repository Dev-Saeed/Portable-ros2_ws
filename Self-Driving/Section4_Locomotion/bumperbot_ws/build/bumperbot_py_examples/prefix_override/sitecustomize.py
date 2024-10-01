import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dev/ros2_ws/Self-Driving/Section4_Locomotion/bumperbot_ws/install/bumperbot_py_examples'
