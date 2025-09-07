import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eleanorlow/spcrob/ros_ws2/install/path_planner'
