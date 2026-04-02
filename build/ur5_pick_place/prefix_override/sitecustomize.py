import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/ros2_ws/src/UR5_Jazzy_Moveit2_Pick_Place/install/ur5_pick_place'
