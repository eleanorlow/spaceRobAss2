import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eleanorlow/spcrob/ros_ws2/build/path_planner/install/path_planner'
