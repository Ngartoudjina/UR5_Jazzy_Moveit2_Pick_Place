import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/beingar/ros2_jazzy_ws/install/hri_manipulation'
